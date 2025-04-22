// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#include "helpers.h"

void ACameraManagerDeleter::operator()(ACameraManager* cameraManager) {
    ACameraManager_delete(cameraManager);
}

void ACameraIdListDeleter::operator()(ACameraIdList* cameraIdList) {
    ACameraManager_deleteCameraIdList(cameraIdList);
}

void ACameraMetadataDeleter::operator()(ACameraMetadata* metadata) {
    ACameraMetadata_free(metadata);
}

void ACameraDeviceDeleter::operator()(ACameraDevice* device) {
    ACameraDevice_close(device);
}

void ACameraOutputTargetDeleter::operator()(ACameraOutputTarget* target) {
    ACameraOutputTarget_free(target);
}

void ACaptureSessionOutputContainerDeleter::operator()(
    ACaptureSessionOutputContainer* outputContainer) {
    ACaptureSessionOutputContainer_free(outputContainer);
}

void ACaptureSessionOutputDeleter::operator()(ACaptureSessionOutput* sessionOutput) {
    ACaptureSessionOutput_free(sessionOutput);
}

void ACameraCaptureSessionDeleter::operator()(ACameraCaptureSession* captureSession) {
    ACameraCaptureSession_close(captureSession);
}

void AImageDeleter::operator()(AImage* image) {
    AImage_delete(image);
}

void AImageReaderDeleter::operator()(AImageReader* imageReader) {
    AImageReader_delete(imageReader);
}

void ANativeWindowDeleter::operator()(ANativeWindow* window) {
    ANativeWindow_release(window);
}
