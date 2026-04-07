#ifndef CAMERA_H
#define CAMERA_H

#include <string>

class Camera {
public:
    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual bool SetSwapBytes(bool enabled) { return false; }  // Optional, default no-op
    virtual std::string Explain(const std::string& question) = 0;
    // Unified capture + explain with mutex to prevent concurrent camera access
    virtual std::string CaptureAndExplain(const std::string& question) = 0;
    // Capture + explain to a custom endpoint path (reusing same host/token)
    virtual std::string CaptureAndExplainToEndpoint(const std::string& endpoint_path, const std::string& question) {
        return CaptureAndExplain(question);
    }
    virtual bool HasExplainUrl() const { return false; }
};

#endif // CAMERA_H
