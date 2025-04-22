// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#include <camera/NdkCameraDevice.h>
#include <camera/NdkCameraManager.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

struct ACameraManagerDeleter {
    void operator()(ACameraManager* _Nonnull cameraManager);
};

struct ACameraIdListDeleter {
    void operator()(ACameraIdList* _Nonnull cameraIdList);
};

struct ACameraMetadataDeleter {
    void operator()(ACameraMetadata* _Nonnull metadata);
};

struct ACameraDeviceDeleter {
    void operator()(ACameraDevice* _Nonnull device);
};

struct ACameraOutputTargetDeleter {
    void operator()(ACameraOutputTarget* _Nonnull target);
};

struct ACaptureSessionOutputContainerDeleter {
    void operator()(ACaptureSessionOutputContainer* _Nonnull outputContainer);
};

struct ACaptureSessionOutputDeleter {
    void operator()(ACaptureSessionOutput* _Nonnull sessionOutput);
};

struct ACameraCaptureSessionDeleter {
    void operator()(ACameraCaptureSession* _Nonnull captureSession);
};

struct AImageDeleter {
    void operator()(AImage* _Nonnull image);
};

struct AImageReaderDeleter {
    void operator()(AImageReader* _Nonnull imageReader);
};

struct ANativeWindowDeleter {
    void operator()(ANativeWindow* _Nonnull window);
};
