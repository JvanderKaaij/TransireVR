// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <variant>
#include <vector>
#include <optional>

class Camera2Device;

struct CameraMetadata {
    using DataEntry = std::variant<uint8_t, int32_t, float, int64_t, double>;
    enum class DataType {
        UINT8_T,
        INT32_T,
        FLOAT,
        INT64_T,
        DOUBLE,
    };
    DataType type;
    std::vector<DataEntry> data;
};

enum class Position : int8_t {Left, Right, Unknown};

struct Camera2Configuration {
    std::string id;
    int32_t width;
    int32_t height;
    float lensTranslation[3];
    float lensRotation[4];
    float fx;
    float fy;
    float cx;
    float cy;
    float s;

    std::optional<bool> isPassthroughCamera;
    Position position = Position::Unknown;
};

class Camera2Manager {
public:
    virtual ~Camera2Manager() = default;

    static std::unique_ptr<Camera2Manager> create();

    virtual std::vector<Camera2Configuration> getCameraConfigs(int targetWidth, int targetHeight) = 0;

    virtual std::shared_ptr<Camera2Device> openCamera(const std::string& id, int targetWidth, int targetHeigth) = 0;

    virtual void shutdown() = 0;

protected:
    Camera2Manager() = default;
};

class Camera2Device {

public:
    virtual ~Camera2Device() = default;

    struct Callbacks {
        std::function<void(int64_t, std::vector<uint8_t>)> onFrame;
        std::function<void(const std::string&)> onError;
    };

    virtual bool start(Callbacks cb) = 0;
    virtual bool stop() = 0;

protected:
    explicit Camera2Device(std::string_view id) : id_(id) {}

    const std::string id_;
};
