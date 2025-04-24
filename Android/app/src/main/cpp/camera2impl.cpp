// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "camera2.h"

#include <atomic>
#include <unordered_map>

#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>
#include <mutex>

#include "ndk/helpers.h"
#include "log.h"

#define ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT 0

// TODO: only one image format is supported yet
static constexpr int kImageFormatAndroid = AIMAGE_FORMAT_YUV_420_888;

static constexpr int kMaxBufCount = 2;

static constexpr int kCameraSourceTag = 0x80004d00;
static constexpr int kCameraPositionTag = kCameraSourceTag + 1;
static constexpr int kCameraSourcePassthrough = 0;
static constexpr int kCameraLeft = 0;
static constexpr int kCameraRight = 1;

class Camera2DeviceImpl;

class Camera2ManagerImpl final : public Camera2Manager {
    friend class Camera2DeviceImpl;

public:
    ~Camera2ManagerImpl() override {
        shutdown();
    }

    void shutdown() override {
        // Clean up all devices
        for (auto& [_, device] : devices_) {
            device->stop();
            device.reset();
        }

        devices_.clear();
    }

    Camera2ManagerImpl() {
        cameraManager_ =
                std::unique_ptr<ACameraManager, ACameraManagerDeleter>(ACameraManager_create());
        if (!cameraManager_) {
            ALOGE(" ***** Camera manager is not initialized");
        }
    }

    std::vector<Camera2Configuration> getCameraConfigs() override {
        if (!cameraManager_) {
            ALOGE(" ***** Camera manager is not initialized");
            return {};
        }

        ACameraIdList* cameraIds{nullptr};
        int ret = ACameraManager_getCameraIdList(cameraManager_.get(), &cameraIds);
        if (ret != ACAMERA_OK || cameraIds == nullptr) {
            ALOGE(" ***** Failed to get camera list");
            return {};
        } else if (cameraIds->numCameras == 0) {
            ALOGE(" ***** No cameras found! Missing permissions?");
            return {};
        }
        auto ids = std::unique_ptr<ACameraIdList, ACameraIdListDeleter>(cameraIds);

        ALOGV("Found %i cameras", cameraIds->numCameras);
        std::vector<Camera2Configuration> res;
        res.reserve(ids->numCameras);
        for (int i = 0; i < ids->numCameras; i++) {
            const char* id = ids->cameraIds[i];
            ALOGV("Querying configuration of camera `%s`", id);

            ACameraMetadata* metadataObj{nullptr};
            ret = ACameraManager_getCameraCharacteristics(cameraManager_.get(), id, &metadataObj);
            if (ret != ACAMERA_OK || metadataObj == nullptr) {
                ALOGE(" ***** Failed to get camera characteristic for camera: `%s`", id);
                continue;
            }
            auto metadata = std::unique_ptr<ACameraMetadata, ACameraMetadataDeleter>(metadataObj);


            Camera2Configuration config;
            config.id = id;
            config.width = 640;
            config.height = 480;

            ACameraMetadata_const_entry entry;
            ret = ACameraMetadata_getConstEntry(metadata.get(), ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);
            if (ret == ACAMERA_OK && entry.count % 4 == 0) {
                for (uint32_t i = 0; i < entry.count; i += 4) {
                    int32_t format = entry.data.i32[i];
                    int32_t width = entry.data.i32[i + 1];
                    int32_t height = entry.data.i32[i + 2];
                    int32_t isOutput = entry.data.i32[i + 3];

                    if (format == AIMAGE_FORMAT_YUV_420_888 && isOutput == ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT && width==480) {//hardcoded TODO fix
                        config.width = width;
                        config.height = height;
                        break;  // just pick the first supported YUV output for now
                    }

                    __android_log_print(ANDROID_LOG_INFO, "AprilTagNative", "Supported resolution: %dx%d (format: %d)", width, height, format);
                }
            }

            CameraMetadata pixelSize =
                    GetCameraMetadata(id, *metadata, (int)ACAMERA_SENSOR_INFO_PIXEL_ARRAY_SIZE);

            if (pixelSize.data.empty()) {
                ALOGE(" ***** No pixel size for camera: `%s`", id);
                continue;
            }

            if (pixelSize.data.size() != 2) {
                ALOGE(" ***** Invalid pixel size for camera: `%s`", id);
                continue;
            }

            if (pixelSize.type != CameraMetadata::DataType::INT32_T) {
                ALOGE(" ***** Invalid data type for pixel size camera: `%s`", id);
                continue;
            }

            CameraMetadata lensPoseTranslation =
                    GetCameraMetadata(id, *metadata, (int)ACAMERA_LENS_POSE_TRANSLATION);
            CameraMetadata lensPoseRotation =
                    GetCameraMetadata(id, *metadata, (int)ACAMERA_LENS_POSE_ROTATION);
            CameraMetadata cameraSource = GetCameraMetadata(id, *metadata, kCameraSourceTag);
            CameraMetadata cameraPosition = GetCameraMetadata(id, *metadata, kCameraPositionTag);

            bool hasLensPose = true;
            if (lensPoseTranslation.data.size() == 0 || lensPoseRotation.data.size() == 0) {
                ALOGE(" ***** No pose data for camera: `%s`", id);
                hasLensPose = false;
            }

//            config.width = std::get<int32_t>(pixelSize.data[0]);
//            config.height = std::get<int32_t>(pixelSize.data[1]);

            if (!cameraSource.data.empty()) {
                config.isPassthroughCamera =
                        std::get<uint8_t>(cameraSource.data[0]) == kCameraSourcePassthrough;
            } else {
                ALOGE(" ***** No camera source for camera: `%s`", id);
            }
            if (!cameraPosition.data.empty()) {
                int32_t position = std::get<uint8_t>(cameraPosition.data[0]);
                if (position == kCameraLeft) {
                    config.position = Position::Left;
                } else if (position == kCameraRight) {
                    config.position = Position::Right;
                } else {
                    ALOGE(" ***** Invalid camera position for camera: `%s`", id);
                }
            } else {
                ALOGE(" ***** No camera position for camera: `%s`", id);
            }

            if (hasLensPose) {
                config.lensTranslation[0] = std::get<float>(lensPoseTranslation.data[0]);
                config.lensTranslation[1] = std::get<float>(lensPoseTranslation.data[1]);
                config.lensTranslation[2] = std::get<float>(lensPoseTranslation.data[2]);

                config.lensRotation[0] = std::get<float>(lensPoseRotation.data[0]);
                config.lensRotation[1] = std::get<float>(lensPoseRotation.data[1]);
                config.lensRotation[2] = std::get<float>(lensPoseRotation.data[2]);
                config.lensRotation[3] = std::get<float>(lensPoseRotation.data[3]);
            } else {
                // Default to identity values
                config.lensTranslation[0] = 0.0f;
                config.lensTranslation[1] = 0.0f;
                config.lensTranslation[2] = 0.0f;

                config.lensRotation[0] = 0.0f;
                config.lensRotation[1] = 0.0f;
                config.lensRotation[2] = 0.0f;
                config.lensRotation[3] = 1.0f;
            }
            res.push_back(config);
        }

        return res;
    }

    static CameraMetadata
    GetCameraMetadata(const std::string& id, ACameraMetadata& metadata, int tag) {
        ACameraMetadata_const_entry entry;
        int ret = ACameraMetadata_getConstEntry(&metadata, tag, &entry);
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to get metadata entry '%d' for camera: `%s`", tag, id.c_str());
            return {};
        }

        CameraMetadata res;
        switch (entry.type) {
            case ACAMERA_TYPE_BYTE:
                res.type = CameraMetadata::DataType::UINT8_T;
                for (uint i = 0; i < entry.count; i++) {
                    res.data.push_back(entry.data.u8[i]);
                }
                break;
            case ACAMERA_TYPE_INT32:
                res.type = CameraMetadata::DataType::INT32_T;
                for (uint i = 0; i < entry.count; i++) {
                    res.data.push_back(entry.data.i32[i]);
                }
                break;
            case ACAMERA_TYPE_FLOAT:
                res.type = CameraMetadata::DataType::FLOAT;
                for (uint i = 0; i < entry.count; i++) {
                    res.data.push_back(entry.data.f[i]);
                }
                break;
            case ACAMERA_TYPE_INT64:
                res.type = CameraMetadata::DataType::INT64_T;
                for (uint i = 0; i < entry.count; i++) {
                    res.data.push_back(entry.data.i64[i]);
                }
                break;
            case ACAMERA_TYPE_DOUBLE:
                res.type = CameraMetadata::DataType::DOUBLE;
                for (uint i = 0; i < entry.count; i++) {
                    res.data.push_back(entry.data.d[i]);
                }
                break;
            default:
                ALOGE(" ***** Unsupported metadata type: `%i`", entry.type);
                return {};
        };

        return res;
    }

    std::shared_ptr<Camera2Device> openCamera(const std::string& id) override;

private:
    std::unique_ptr<ACameraManager, ACameraManagerDeleter> cameraManager_{};
    std::unordered_map<std::string, std::shared_ptr<Camera2Device>> devices_{};
};

class Camera2DeviceImpl final : public Camera2Device {
    friend class Camera2ManagerImpl;

public:
    ~Camera2DeviceImpl() override {
        // Order is important here

        // Remove from manager
        camera2Manager_.devices_.erase(id_);

        // Clean listeners, there is no way to disable them
        imageListener_.context = nullptr;
        imageListener_.onImageAvailable = nullptr;
        AImageReader_setImageListener(imageReader_.get(), &imageListener_);

        if (captureRequest_) {
            ACaptureRequest_removeTarget(captureRequest_, cameraOutputTarget_.get());
            ACaptureRequest_free(captureRequest_);
            captureRequest_ = nullptr;
        }

        cameraOutputTarget_.reset();

        if (captureSessionOutput_) {
            ACaptureSessionOutputContainer_remove(
                    captureSessionOutputContainer_.get(), captureSessionOutput_.get());
            captureSessionOutput_.reset();
        }

        imageReaderWindow_.reset();
        imageReader_.reset();

        captureSessionOutputContainer_.reset();

        cameraDevice_.reset();
    };

    Camera2DeviceImpl(std::string_view id, Camera2ManagerImpl& cameraManager)
            : Camera2Device(id), camera2Manager_(cameraManager) {}

    bool create(Camera2Configuration config) {
        auto* manager = camera2Manager_.cameraManager_.get();
        if (manager == nullptr) {
            ALOGE(" ***** Invalid camera manager");
            return false;
        }

        cameraDeviceCallbacks_ = {
                .context = this,
                .onDisconnected = [](void* /* context */,
                                     ACameraDevice* /* device */) { ALOGE("Camera disconnected"); },
                .onError = [](void* /* context */,
                              ACameraDevice* /* device */,
                              int error) { ALOGE("Camera error: %i", error); },
        };

        ACameraDevice* device{nullptr};
        int ret = ACameraManager_openCamera(manager, id_.data(), &cameraDeviceCallbacks_, &device);
        if (ret != ACAMERA_OK || device == nullptr) {
            ALOGE(" ***** Failed to open camera: `%s`", id_.c_str());
            return false;
        }

        cameraDevice_ = std::unique_ptr<ACameraDevice, ACameraDeviceDeleter>(device);

        ACaptureSessionOutputContainer* captureSessionOutputContainer{nullptr};
        ret = ACaptureSessionOutputContainer_create(&captureSessionOutputContainer);
        if (ret != ACAMERA_OK && captureSessionOutputContainer == nullptr) {
            ALOGE(" ***** Failed to create capture session output container");
            return false;
        }
        captureSessionOutputContainer_ =
                std::unique_ptr<ACaptureSessionOutputContainer, ACaptureSessionOutputContainerDeleter>(
                        captureSessionOutputContainer);

        AImageReader* imageReader{nullptr};
        ret = AImageReader_new(
                config.width, config.height, kImageFormatAndroid, kMaxBufCount, &imageReader);

        if (ret != ACAMERA_OK || imageReader == nullptr) {
            ALOGE(" ***** Failed to create image reader");
            return false;
        }
        imageReader_ = std::unique_ptr<AImageReader, AImageReaderDeleter>(imageReader);

        imageListener_.context = this;
        imageListener_.onImageAvailable = &Camera2DeviceImpl::onImageAvailable;

        ret = AImageReader_setImageListener(imageReader_.get(), &imageListener_);
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to set image listener");
            return false;
        }

        ANativeWindow* imageReaderWindow{nullptr};
        ret = AImageReader_getWindow(imageReader_.get(), &imageReaderWindow);
        if (ret != ACAMERA_OK || imageReaderWindow == nullptr) {
            ALOGE(" ***** Failed to get image reader window");
            return false;
        }
        imageReaderWindow_ =
                std::unique_ptr<ANativeWindow, ANativeWindowDeleter>(imageReaderWindow);
        ANativeWindow_acquire(imageReaderWindow_.get());

        ACameraOutputTarget* cameraOutputTarget;
        ret = ACameraOutputTarget_create(imageReaderWindow_.get(), &cameraOutputTarget);
        if (ret != ACAMERA_OK || cameraOutputTarget == nullptr) {
            ALOGE(" ***** Failed to create camera output target");
            return false;
        }
        cameraOutputTarget_ =
                std::unique_ptr<ACameraOutputTarget, ACameraOutputTargetDeleter>(cameraOutputTarget);

        ret = ACameraDevice_createCaptureRequest(
                cameraDevice_.get(), TEMPLATE_RECORD, &captureRequest_);
        if (ret != ACAMERA_OK || captureRequest_ == nullptr) {
            ALOGE(" ***** Failed to create capture request");
            return false;
        }

        ret = ACaptureRequest_addTarget(captureRequest_, cameraOutputTarget_.get());
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to add target to capture request");
            return false;
        }

        // This is the ampount of time in nanoseconds between each frame or the inverse of the frame
        // rate So for 30fps this value should be 33333333ns; for 60fps it should be 16666666ns
        const int64_t value = 33333333;
        ret =
                ACaptureRequest_setEntry_i64(captureRequest_, ACAMERA_SENSOR_FRAME_DURATION, 1, &value);
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to set frame duration %i", ret);
            return false;
        }

        ACaptureSessionOutput* captureSessionOutput{nullptr};
        ret = ACaptureSessionOutput_create(imageReaderWindow_.get(), &captureSessionOutput);
        if (ret != ACAMERA_OK || captureSessionOutput == nullptr) {
            ALOGE(" ***** Failed to create capture session output");
            return false;
        }
        captureSessionOutput_ =
                std::unique_ptr<ACaptureSessionOutput, ACaptureSessionOutputDeleter>(
                        captureSessionOutput);

        ret = ACaptureSessionOutputContainer_add(
                captureSessionOutputContainer_.get(), captureSessionOutput_.get());
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to add capture session output to container");
            return false;
        }

        return true;
    }

    bool start(Callbacks cb) override {
        std::unique_lock<std::mutex> lock(runningMutex_);
        if (captureRequest_ == nullptr) {
            ALOGE(" ***** Capture request is not created");
            return false;
        }

        if (captureSession_.get() != nullptr) {
            ALOGE(" ***** Capture session is already created");
            return false;
        }

        captureSessionStateCallbacks_.context = nullptr;
        captureSessionStateCallbacks_.onClosed = [](void* /*context*/,
                                                    ACameraCaptureSession* session) {
            ALOGV(" ***** onClosed: %p", session);
        };
        captureSessionStateCallbacks_.onReady = [](void* /*context*/,
                                                   ACameraCaptureSession* session) {
            ALOGV(" ***** onReady: %p", session);
        };
        captureSessionStateCallbacks_.onActive = [](void* /*context*/,
                                                    ACameraCaptureSession* session) {
            ALOGV(" ***** onActive: %p", session);
        };

        ALOGE(" ***** Create capture session ****");
        ACameraCaptureSession* captureSession;
        int ret = ACameraDevice_createCaptureSession(
                cameraDevice_.get(),
                captureSessionOutputContainer_.get(),
                &captureSessionStateCallbacks_,
                &captureSession);

        if (ret != ACAMERA_OK || captureSession == nullptr) {
            ALOGE(" ***** Failed to create capture session");
            return false;
        }
        captureSession_ =
                std::unique_ptr<ACameraCaptureSession, ACameraCaptureSessionDeleter>(captureSession);

        callbacks_ = cb;

        ALOGE(" ***** Set repeating request");
        ret = ACameraCaptureSession_setRepeatingRequest(
                captureSession_.get(), nullptr, 1, &captureRequest_, nullptr);
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to set repeating request");
            captureSession_.reset();
            return false;
        }

        isRunning_ = true;

        return true;
    }

    bool stop() override {
        std::unique_lock<std::mutex> lock(runningMutex_);

        if (!isRunning_) {
            ALOGE(" ***** Capture is not started");
            return false;
        }

        isRunning_ = false;

        int ret = ACameraCaptureSession_stopRepeating(captureSession_.get());
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to stop repeating");
        } else {
            ALOGE(" ***** Stopped repeating request");
        }

        ret = ACameraCaptureSession_abortCaptures(captureSession_.get());
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to abort captures");
        } else {
            ALOGE(" ***** Aborted captures");
        }

        captureSession_.reset();

        return true;
    }

private:
    // TODO: supports only NV21 and NV12 yet
    std::vector<uint8_t> convertMessageToBuffer(AImage* image) {
        int32_t image_pixelstrides[2];
        uint8_t* image_plane_data[2];
        int plane_data_length[2];

        for (int i = 0; i < 2; i++) {
            AImage_getPlanePixelStride(image, i + 1, &image_pixelstrides[i]);
            AImage_getPlaneData(image, i + 1, &image_plane_data[i], &plane_data_length[i]);
        }

        if (image_pixelstrides[0] != image_pixelstrides[1]) {
            ALOGE(" ***** Pixel strides of U and V plane should have been the same");
            return {};
        }

        switch (image_pixelstrides[0]) {
            case 1:
                ALOGE(" ***** YUV420P is not supported yet");
                return {};
            case 2:
                if (image_plane_data[0] < image_plane_data[1]) {
                    ALOGE(" ***** Image format: NV12");
                } else {
                    ALOGE(" ***** Image format: NV21");
                }
                break;
            default:
                ALOGE(
                        " ***** Unknown pixel stride %i of U and V plane, cannot determine camera image format",
                        image_pixelstrides[0]);
                return {};
        }

        uint8_t *yBuffer, *uBuffer, *vBuffer;
        int width, height;
        int yLen, uvLen;
        int yPixelStride, yRowStride, uvPixelStride, uvRowStride;
        AImage_getPlaneData(image, 0, &yBuffer, &yLen);
        AImage_getPlanePixelStride(image, 0, &yPixelStride);
        AImage_getPlaneRowStride(image, 0, &yRowStride);

        AImage_getPlaneData(image, 1, &uBuffer, &uvLen);
        AImage_getPlanePixelStride(image, 1, &uvPixelStride);
        AImage_getPlaneRowStride(image, 1, &uvRowStride);

        AImage_getPlaneData(image, 2, &vBuffer, &uvLen);

        AImage_getWidth(image, &width);
        AImage_getHeight(image, &height);

        ALOGV(" ***** Image size: %ix%i", width, height);
        ALOGV(
                " ***** Y plane: %ix%i, stride: %i, pixel stride: %i",
                yRowStride,
                yLen / yRowStride,
                yRowStride,
                yPixelStride);
        ALOGV(
                " ***** UV plane: %ix%i, stride: %i, pixel stride: %i",
                uvRowStride,
                uvLen / uvRowStride,
                uvRowStride,
                uvPixelStride);

        const size_t size = yLen + 2 * uvLen;
        std::vector<uint8_t> buffer;
        buffer.resize(size);
        memcpy(buffer.data(), yBuffer, yLen);
        memcpy(buffer.data() + yLen, vBuffer, uvLen);
        memcpy(buffer.data() + yLen + uvLen, uBuffer, uvLen);

        return buffer;
    }

    static void onImageAvailable(void* ctx, AImageReader* reader) {
        if (ctx == nullptr) {
            ALOGE("Invalid context");
            return;
        }
        auto* self = reinterpret_cast<Camera2DeviceImpl*>(ctx);

        std::unique_lock<std::mutex> lock(self->runningMutex_);

        ALOGE(" ***** onImageAvailable");

        // Read the image to prevent blocking
        AImage* image{nullptr};
        int ret = AImageReader_acquireLatestImage(reader, &image);
        if (ret != ACAMERA_OK) {
            ALOGE(" ***** Failed to acquire latest image");
            return;
        }
        if (!self->isRunning_) {
            ALOGE(" ***** Capture is not started, skipping image");
            AImage_delete(image);
            return;
        }

        int64_t imageTimestamp{-1};

        AImage_getTimestamp(image, &imageTimestamp);
        ALOGE(" ***** Image timestamp: %lld *****", imageTimestamp);

        auto buf = self->convertMessageToBuffer(image);
        AImage_delete(image);

        if (!buf.empty() && self->callbacks_.onFrame) {
            self->callbacks_.onFrame(imageTimestamp, buf);
        }
    };

private:
    Callbacks callbacks_{};

    Camera2ManagerImpl& camera2Manager_;

    std::atomic_bool isRunning_{false};
    std::mutex runningMutex_;

    // Android Camera2 API objects
    std::unique_ptr<ACameraDevice, ACameraDeviceDeleter> cameraDevice_{nullptr};
    std::unique_ptr<ACaptureSessionOutputContainer, ACaptureSessionOutputContainerDeleter>
            captureSessionOutputContainer_{nullptr};
    std::unique_ptr<AImageReader, AImageReaderDeleter> imageReader_{nullptr};
    std::unique_ptr<ANativeWindow, ANativeWindowDeleter> imageReaderWindow_{nullptr};
    std::unique_ptr<ACameraOutputTarget, ACameraOutputTargetDeleter> cameraOutputTarget_{nullptr};
    std::unique_ptr<ACaptureSessionOutput, ACaptureSessionOutputDeleter> captureSessionOutput_{
            nullptr};
    std::unique_ptr<ACameraCaptureSession, ACameraCaptureSessionDeleter> captureSession_{nullptr};

    ACaptureRequest* captureRequest_{nullptr};

    // Android Camera2 API callbacks
    AImageReader_ImageListener imageListener_;
    ACameraDevice_stateCallbacks cameraDeviceCallbacks_;
    ACameraCaptureSession_stateCallbacks captureSessionStateCallbacks_;
};

std::shared_ptr<Camera2Device> Camera2ManagerImpl::openCamera(const std::string& id) {
    std::string idStr(id);

    if (devices_.find(idStr) != devices_.end()) {
        ALOGE(" ***** Camera `%s` is already opened", id.data());
        return nullptr;
    }

    auto configs = getCameraConfigs();
    auto cameraConfig =
            std::find_if(configs.begin(), configs.end(), [id](const Camera2Configuration& config) {
                return config.id == id;
            });
    if (cameraConfig == configs.end()) {
        ALOGE(" ***** Camera `%s` is not found", id.data());
        return nullptr;
    }

    auto deviceImpl = new Camera2DeviceImpl(id, *this);
    if (deviceImpl != nullptr && !deviceImpl->create(*cameraConfig)) {
        return nullptr;
    }
    auto device = std::shared_ptr<Camera2Device>(deviceImpl);

    devices_[std::string(id)] = device;
    return device;
}

std::unique_ptr<Camera2Manager> Camera2Manager::create() {
    return std::make_unique<Camera2ManagerImpl>();
}
