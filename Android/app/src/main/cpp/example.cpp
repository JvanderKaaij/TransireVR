// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include <cstddef>
#include <cstdio>
#include <ctime>
#include <memory>
#include <string>
#include <vector>

#include "camera2.h"
#include "XrApp.h"
#include "Input/TinyUI.h"
#include "Input/ControllerRenderer.h"
#include "Misc/Log.h"
#include "Render/SimpleBeamRenderer.h"

#include <meta_openxr_preview/openxr_oculus_helpers.h>
#include <openxr/openxr.h>

#define DECLARE_FUNCTION_PTR(functionName) PFN_##functionName functionName = nullptr;
#define HOOKUP_FUNCTION_PTR(functionName) \
    OXR(xrGetInstanceProcAddr(GetInstance(), #functionName, (PFN_xrVoidFunction*)(&functionName)));
#define VALIDATE_FUNCTION_PTR(functionName)                                    \
    if (functionName == nullptr) {                                             \
        ALOGE("Skipping: this app does not currently support " #functionName); \
        return;                                                                \
    }
#define K_MAX_PERSISTENT_SPACES 32

class XrCameraApp : public OVRFW::XrApp {
   public:
    XrCameraApp() : OVRFW::XrApp() {
        BackgroundColor = OVR::Vector4f(0.0f, 0.0f, 0.0f, 0.0f);
    }

    // Must return true if the application initializes successfully.
    virtual bool AppInit(const xrJava* context) override {
        if (false == Ui.Init(context, GetFileSys())) {
            ALOGE("TinyUI::Init FAILED.");
            return false;
        }

        HookExtensions();
        CreateButtonsUI();

        return true;
    }

   private:
    void StartCameraButtonPressed() {
        if (activeCamera == nullptr) {
            activeCamera = OpenCamera(Position::Right);
        }

        Camera2Device::Callbacks callbacks{
            std::bind(
                &XrCameraApp::onFrameCallback, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&XrCameraApp::onErrorCallback, this, std::placeholders::_1)};

        if (!activeCamera) {
            ALOGE("***** Camera Device is NULL. Unable to start. *****");
            return;
        }

        auto started = activeCamera->start(callbacks);
        ALOGE(" ***** Started Camera %i *****", started);
    }

    void StopCameraButtonPressed() {
        if (activeCamera.get() != nullptr) {
            activeCamera->stop();
        }
    }

    void ExitButtonPressed() {
        ShutdownCameraManager();
        exit(0);
    }

    virtual void AppShutdown(const xrJava* context) override {
        ALOGE(" ***** App shutdown *****");
        Ui.Shutdown();
        OVRFW::XrApp::AppShutdown(context);
    }

    void CreateButtonsUI() {
        const OVR::Vector2f buttonSize = {300.0f, 100.0f};
        const float buttonXPos = -1.0f;
        const float buttonZPos = -2.0f;

        StartCameraButton =
            Ui.AddButton("Start Camera", {buttonXPos, 1.9f, buttonZPos}, buttonSize, [this]() {
                StartCameraButtonPressed();
            });
        StopCameraButton =
            Ui.AddButton("Stop Camera", {buttonXPos, 1.6f, buttonZPos}, buttonSize, [this]() {
                StopCameraButtonPressed();
            });
        ExitButton = Ui.AddButton(
            "Exit", {buttonXPos, 1.3f, buttonZPos}, buttonSize, [this]() { ExitButtonPressed(); });

        const float labelDescriptionXPos = -0.25f;
        const OVR::Vector2f labelDescriptionSize = {200.0f, 75.0f};
        const float labelValueXPos = 0.5f;
        const OVR::Vector2f labelValueSize = {500.0f, 75.0f};

        AverageBrightnessDescriptionLabel = Ui.AddLabel(
            "Average\nBrightness", {labelDescriptionXPos, 1.9f, buttonZPos}, labelDescriptionSize);
        AverageBrightnessDescriptionLabel->SetColor({0.0f, 0.0f, 0.0f, 1.0f});
        CameraPosDescriptionLabel = Ui.AddLabel(
            "Camera Pose\n(x,y,z)", {labelDescriptionXPos, 1.6f, buttonZPos}, labelDescriptionSize);
        CameraPosDescriptionLabel->SetColor({0.0f, 0.0f, 0.0f, 1.0f});
        CameraRotDescriptionLabel = Ui.AddLabel(
            "Camera Rotation\n(x,y,z,w)",
            {labelDescriptionXPos, 1.3f, buttonZPos},
            labelDescriptionSize);
        CameraRotDescriptionLabel->SetColor({0.0f, 0.0f, 0.0f, 1.0f});

        AverageBrightnessValueLabel =
            Ui.AddLabel("", {labelValueXPos, 1.9f, buttonZPos}, labelValueSize);
        AverageBrightnessValueLabel->SetColor({0.50f, 0.50f, 0.50f, 1.0f});
        CameraPosValueLabel = Ui.AddLabel("", {labelValueXPos, 1.6f, buttonZPos}, labelValueSize);
        CameraPosValueLabel->SetColor({0.50f, 0.50f, 0.50f, 1.0f});
        CameraRotValueLabel = Ui.AddLabel("", {labelValueXPos, 1.3f, buttonZPos}, labelValueSize);
        CameraRotValueLabel->SetColor({0.50f, 0.50f, 0.50f, 1.0f});
    }

    virtual void PreProjectionAddLayer(xrCompositorLayerUnion* layers, int& layerCount) override {
        AddPassthroughLayer(layers, layerCount);
    }

    // Returns a list of OpenXR extensions requested for this app
    // Note that the sample framework will filter out any extension
    // that is not listed as supported.
    virtual std::vector<const char*> GetExtensions() override {
        std::vector<const char*> extensions = XrApp::GetExtensions();
        extensions.push_back(XR_FB_PASSTHROUGH_EXTENSION_NAME);

        return extensions;
    }

    void AddPassthroughLayer(xrCompositorLayerUnion* layers, int& layerCount) {
        if (PassthroughLayer != XR_NULL_HANDLE) {
            XrCompositionLayerPassthroughFB passthroughLayer = {
                XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB};
            passthroughLayer.layerHandle = PassthroughLayer;
            passthroughLayer.flags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
            passthroughLayer.space = XR_NULL_HANDLE;
            layers[layerCount++].Passthrough = passthroughLayer;
        }
    }

    virtual bool SessionInit() override {
        if (!CreateControllerRenderer())
            return false;

        StartPassthrough();
        CreateCameraManager();

        return true;
    }

    bool CreateControllerRenderer() {
        /// Init session bound objects
        if (false == ControllerRenderL.Init(true)) {
            ALOGE("AppInit::Init L controller renderer FAILED.");
            return false;
        }
        if (false == ControllerRenderR.Init(false)) {
            ALOGE("AppInit::Init R controller renderer FAILED.");
            return false;
        }
        BeamRenderer.Init(GetFileSys(), nullptr, OVR::Vector4f(1.0f), 1.0f);

        return true;
    }

    void StartPassthrough() {
        XrPassthroughCreateInfoFB ptci{XR_TYPE_PASSTHROUGH_CREATE_INFO_FB};
        XrResult result;
        OXR(result = xrCreatePassthroughFB(GetSession(), &ptci, &Passthrough));

        if (XR_SUCCEEDED(result)) {
            ALOGV("Creating passthrough layer");
            XrPassthroughLayerCreateInfoFB plci{XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB};
            plci.passthrough = Passthrough;
            plci.purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;
            OXR(xrCreatePassthroughLayerFB(GetSession(), &plci, &PassthroughLayer));

            ALOGV("Setting passthrough style");
            XrPassthroughStyleFB style{XR_TYPE_PASSTHROUGH_STYLE_FB};
            OXR(xrPassthroughLayerResumeFB(PassthroughLayer));
            style.textureOpacityFactor = 0.5f;
            style.edgeColor = {0.0f, 0.0f, 0.0f, 0.0f};
            OXR(xrPassthroughLayerSetStyleFB(PassthroughLayer, &style));
        } else {
            ALOGV("Create passthrough failed");
        }
        if (result != XR_ERROR_FEATURE_UNSUPPORTED) {
            OXR(result = xrPassthroughStartFB(Passthrough));
        }
    }

    virtual void SessionEnd() override {
        ALOGE(" ***** Session End *****");
        ControllerRenderL.Shutdown();
        ControllerRenderR.Shutdown();
        BeamRenderer.Shutdown();
    }

    // Update state
    virtual void Update(const OVRFW::ovrApplFrameIn& in) override {
        Ui.HitTestDevices().clear();

        if (in.LeftRemoteTracked) {
            ControllerRenderL.Update(in.LeftRemotePose);
            const bool didPinch = in.LeftRemoteIndexTrigger > 0.5f;
            Ui.AddHitTestRay(in.LeftRemotePointPose, didPinch);
        }
        if (in.RightRemoteTracked) {
            ControllerRenderR.Update(in.RightRemotePose);
            const bool didPinch = in.RightRemoteIndexTrigger > 0.5f;
            Ui.AddHitTestRay(in.RightRemotePointPose, didPinch);
        }

        Ui.Update(in);
        BeamRenderer.Update(in, Ui.HitTestDevices());

        if (in.Clicked(in.kButtonA)) {
            ALOGE("\'A\' button is clicked, creating a shared anchor");
        }
    }

    // Render eye buffers while running
    virtual void Render(const OVRFW::ovrApplFrameIn& in, OVRFW::ovrRendererOutput& out) override {
        /// Render UI
        Ui.Render(in, out);

        /// Render controllers
        if (in.LeftRemoteTracked) {
            ControllerRenderL.Render(out.Surfaces);
        }
        if (in.RightRemoteTracked) {
            ControllerRenderR.Render(out.Surfaces);
        }

        /// Render beams last, since they render with transparency (alpha blending)
        BeamRenderer.Render(in, out);
    }

    virtual void HandleXrEvents() override {
        XrEventDataBuffer eventDataBuffer = {};

        // Poll for events
        for (;;) {
            XrEventDataBaseHeader* baseEventHeader = (XrEventDataBaseHeader*)(&eventDataBuffer);
            baseEventHeader->type = XR_TYPE_EVENT_DATA_BUFFER;
            baseEventHeader->next = NULL;
            XrResult r;
            OXR(r = xrPollEvent(Instance, &eventDataBuffer));
            if (r != XR_SUCCESS) {
                break;
            }

            switch (baseEventHeader->type) {
                case XR_TYPE_EVENT_DATA_EVENTS_LOST:
                    ALOGV("xrPollEvent: received XR_TYPE_EVENT_DATA_EVENTS_LOST event");
                    break;
                case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
                    ALOGV("xrPollEvent: received XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING event");
                    break;
                case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
                    ALOGV(
                        "xrPollEvent: received XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED event");
                    break;
                case XR_TYPE_EVENT_DATA_PERF_SETTINGS_EXT: {
                    const XrEventDataPerfSettingsEXT* perfSettingsEvent =
                        (XrEventDataPerfSettingsEXT*)(baseEventHeader);
                    ALOGV(
                        "xrPollEvent: received XR_TYPE_EVENT_DATA_PERF_SETTINGS_EXT event: type %d subdomain %d : level %d -> level %d",
                        perfSettingsEvent->type,
                        perfSettingsEvent->subDomain,
                        perfSettingsEvent->fromLevel,
                        perfSettingsEvent->toLevel);
                } break;
                case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
                    ALOGV(
                        "xrPollEvent: received XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING event");
                    break;
                case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                    const XrEventDataSessionStateChanged* sessionStateChangedEvent =
                        (XrEventDataSessionStateChanged*)(baseEventHeader);
                    ALOGV(
                        "xrPollEvent: received XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: %d for session %p at time %f",
                        sessionStateChangedEvent->state,
                        (void*)sessionStateChangedEvent->session,
                        FromXrTime(sessionStateChangedEvent->time));

                    switch (sessionStateChangedEvent->state) {
                        case XR_SESSION_STATE_FOCUSED:
                            Focused = true;
                            break;
                        case XR_SESSION_STATE_VISIBLE:
                            Focused = false;
                            break;
                        case XR_SESSION_STATE_READY:
                            HandleSessionStateChanges(sessionStateChangedEvent->state);
                            break;
                        case XR_SESSION_STATE_STOPPING:
                            HandleSessionStateChanges(sessionStateChangedEvent->state);
                            break;
                        case XR_SESSION_STATE_EXITING:
                            ShouldExit = true;
                            break;
                        default:
                            break;
                    }
                } break;

                default:
                    ALOGV("xrPollEvent: Unknown event");
                    break;
            }
        }
    }

    void HookExtensions() {
        /// Passthrough
        HOOKUP_FUNCTION_PTR(xrCreatePassthroughFB)
        HOOKUP_FUNCTION_PTR(xrDestroyPassthroughFB)
        HOOKUP_FUNCTION_PTR(xrCreatePassthroughLayerFB)
        HOOKUP_FUNCTION_PTR(xrDestroyPassthroughLayerFB)
        HOOKUP_FUNCTION_PTR(xrPassthroughLayerResumeFB)
        HOOKUP_FUNCTION_PTR(xrPassthroughLayerPauseFB)
        HOOKUP_FUNCTION_PTR(xrPassthroughLayerSetStyleFB)
        HOOKUP_FUNCTION_PTR(xrPassthroughStartFB)
        HOOKUP_FUNCTION_PTR(xrPassthroughPauseFB)

        // Location
        HOOKUP_FUNCTION_PTR(xrLocateSpace)
    }

   private:
    // Passthrough layer
    XrPassthroughFB Passthrough = XR_NULL_HANDLE;
    XrPassthroughLayerFB PassthroughLayer = XR_NULL_HANDLE;

    // Passthrough
    DECLARE_FUNCTION_PTR(xrCreatePassthroughFB)
    DECLARE_FUNCTION_PTR(xrDestroyPassthroughFB)
    DECLARE_FUNCTION_PTR(xrCreatePassthroughLayerFB)
    DECLARE_FUNCTION_PTR(xrDestroyPassthroughLayerFB)
    DECLARE_FUNCTION_PTR(xrPassthroughLayerResumeFB)
    DECLARE_FUNCTION_PTR(xrPassthroughLayerPauseFB)
    DECLARE_FUNCTION_PTR(xrPassthroughLayerSetStyleFB)
    DECLARE_FUNCTION_PTR(xrPassthroughStartFB)
    DECLARE_FUNCTION_PTR(xrPassthroughPauseFB)

    // Location
    DECLARE_FUNCTION_PTR(xrLocateSpace)

    OVRFW::ControllerRenderer ControllerRenderL;
    OVRFW::ControllerRenderer ControllerRenderR;
    OVRFW::TinyUI Ui;
    OVRFW::SimpleBeamRenderer BeamRenderer;
    std::vector<OVRFW::ovrBeamRenderer::handle_t> Beams;

    // UI Menu
    OVRFW::VRMenuObject* StartCameraButton;
    OVRFW::VRMenuObject* StopCameraButton;
    OVRFW::VRMenuObject* ExitButton;
    OVRFW::VRMenuObject* AverageBrightnessDescriptionLabel;
    OVRFW::VRMenuObject* CameraPosDescriptionLabel;
    OVRFW::VRMenuObject* CameraRotDescriptionLabel;
    OVRFW::VRMenuObject* AverageBrightnessValueLabel;
    OVRFW::VRMenuObject* CameraPosValueLabel;
    OVRFW::VRMenuObject* CameraRotValueLabel;

    //
    // Camera2 API usage
    //
    std::unique_ptr<Camera2Manager> CameraManager{nullptr};
    std::vector<Camera2Configuration> CameraConfigs;

    void CreateCameraManager() {
        CameraManager = Camera2Manager::create();
        CameraConfigs = CameraManager->getCameraConfigs();

        for (const Camera2Configuration& config : CameraConfigs) {
            ALOGV(" ***** Camera ID ****** %s", config.id.c_str());
            ALOGV(" ***** Width ****** %i", config.width);
            ALOGV(" ***** Height ****** %i\n", config.height);
            ALOGV(
                " ***** Lens Translation ****** %f %f %f \n",
                config.lensTranslation[0],
                config.lensTranslation[1],
                config.lensTranslation[2]);
            ALOGV(
                " ***** Lens Rotation ****** %f %f %f %f \n",
                config.lensRotation[0],
                config.lensRotation[1],
                config.lensRotation[2],
                config.lensRotation[3]);
            ALOGV(
                " ***** Handedness ****** %s \n",
                config.position == Position::Unknown    ? "unknown"
                    : config.position == Position::Left ? "Left"
                                                        : "Right");
            ALOGV(
                " ***** is passthrough camera ****** %i \n",
                config.isPassthroughCamera.has_value()
                    ? static_cast<int>(config.isPassthroughCamera.value())
                    : -1);
            ALOGV(
                " ***** Lens Rotation ****** %f %f %f %f \n",
                config.lensRotation[0],
                config.lensRotation[1],
                config.lensRotation[2],
                config.lensRotation[3]);
        }
    }

    void ShutdownCameraManager() {
        CameraManager->shutdown();
        CameraManager.reset();
    }

    std::shared_ptr<Camera2Device> OpenCamera(Position position) {
        if (CameraConfigs.empty()) {
            ALOGE("No Camera Configs found. Unable to open camera. Did you grant permissions?");
            return nullptr;
        }
        // Find `id` matching the given position.
        const auto it = std::find_if(
            CameraConfigs.begin(),
            CameraConfigs.end(),
            [position](const Camera2Configuration& config) { return config.position == position; });

        if (it == CameraConfigs.end()) {
            ALOGE("Invalid Camera Position. Unable to open camera.");
            return nullptr;
        }

        ALOGE(" ***** Opening Camera ****** %s", it->id.c_str());
        activeCameraConfig = *it;

        return CameraManager->openCamera(it->id);
    }

    //
    // Camera2 API Callbacks
    //
    void onFrameCallback(int64_t timestamp, std::vector<uint8_t> imageData) {
        XrResult result;
        XrSpaceLocation location{XR_TYPE_SPACE_LOCATION};
        OXR(result = xrLocateSpace(GetHeadSpace(), GetLocalSpace(), timestamp, &location));

        UpdateDisplayAverageBrightness(
            imageData, activeCameraConfig.width, activeCameraConfig.height);
        UpdateDisplayPose(location);
    }

    void onErrorCallback(const std::string& error) {
        ALOGE(" ***** Error: *****", error.c_str());
    }

    void
    UpdateDisplayAverageBrightness(const std::vector<uint8_t>& yuvData, int width, int height) {
        uint32_t sumY = 0;

        // Iterate over each pixel in the YUV data
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                // Calculate the index of the current pixel in the YUV data
                size_t idx = (i * width) + j;
                sumY += yuvData[idx];
            }
        }

        char label[1000];
        std::snprintf(label, 999, "%i", static_cast<uint8_t>(sumY / (width * height)));
        AverageBrightnessValueLabel->SetText(label);
    }

    void UpdateDisplayPose(const XrSpaceLocation& location) {
        char label[100];

        OVR::Quatf trackedHMDRotation = OVR::Quatf(
            location.pose.orientation.x,
            location.pose.orientation.y,
            location.pose.orientation.z,
            location.pose.orientation.w);
        OVR::Vector3f trackedHMDPosition = OVR::Vector3f(
            location.pose.position.x, location.pose.position.y, location.pose.position.z);
        OVR::Posef trackedHMDPose = OVR::Posef(trackedHMDRotation, trackedHMDPosition);

        OVR::Quatf cameraOffsetRotation = OVR::Quatf(
            activeCameraConfig.lensRotation[0],
            activeCameraConfig.lensRotation[1],
            activeCameraConfig.lensRotation[2],
            activeCameraConfig.lensRotation[3]);
        OVR::Vector3f cameraOffsetPosition = OVR::Vector3f(
            activeCameraConfig.lensTranslation[0],
            activeCameraConfig.lensTranslation[1],
            activeCameraConfig.lensTranslation[2]);
        OVR::Posef cameraOffsetPose = OVR::Posef(cameraOffsetRotation, cameraOffsetPosition);

        // Multiply the camera offset by the tracked HMD pose to get the final camera pose
        OVR::Posef cameraPose = trackedHMDPose * cameraOffsetPose;

        std::snprintf(
            label,
            99,
            "%f %f %f",
            cameraPose.Translation.x,
            cameraPose.Translation.y,
            cameraPose.Translation.z);
        CameraPosValueLabel->SetText(label);

        std::snprintf(
            label,
            99,
            "%f %f %f %f",
            cameraPose.Rotation.x,
            cameraPose.Rotation.y,
            cameraPose.Rotation.z,
            cameraPose.Rotation.w);
        CameraRotValueLabel->SetText(label);
    }

    std::shared_ptr<Camera2Device> activeCamera = nullptr;
    Camera2Configuration activeCameraConfig;
};

ENTRY_POINT(XrCameraApp)
