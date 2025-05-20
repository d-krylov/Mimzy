#ifndef MIMZY_EXAMPLES_TOOLS_H
#define MIMZY_EXAMPLES_TOOLS_H

#include <cstdint>
#include <span>

struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

class Renderer {
public:
  Renderer(int32_t width, int32_t height);

  ~Renderer();

  void PollEvent();
  void Clear();
  void Present();
  void DrawImage(std::span<const uint32_t> image);

  bool ShouldClose() const {
    return quit_;
  }

private:
  SDL_Window *native_window_;
  SDL_Renderer *native_renderer_;
  SDL_Texture *texture_;
  bool quit_{false};
};

#endif // MIMZY_EXAMPLES_TOOLS_H