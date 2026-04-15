#ifndef CAMERA_H
#define CAMERA_H

#include <string>

class Camera {
public:
    virtual ~Camera() = default;
    virtual void SetExplainUrl(const std::string& url, const std::string& token,
                               const std::string& session_id = "") = 0;
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }
    virtual std::string Explain(const std::string& question) = 0;
    virtual std::string CaptureAndExplain(const std::string& question) = 0;
    virtual std::string CaptureAndExplainToEndpoint(const std::string& endpoint_path,
                                                    const std::string& question) {
        return CaptureAndExplain(question);
    }
    virtual bool HasExplainUrl() const { return false; }
    virtual bool IsAvailable() const { return false; }
};

#endif // CAMERA_H
