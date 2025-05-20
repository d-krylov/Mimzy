#include "tools.h"
#include <SDL3/SDL.h>

Renderer::Renderer(int32_t width, int32_t height) {
  SDL_Init(SDL_INIT_VIDEO);
  SDL_CreateWindowAndRenderer("Viewer", width, height, 0, &native_window_, &native_renderer_);
  texture_ = SDL_CreateTexture(native_renderer_, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, width, height);
}

Renderer::~Renderer() {
  SDL_DestroyRenderer(native_renderer_);
  SDL_DestroyWindow(native_window_);
  SDL_Quit();
}

void Renderer::Clear() {
  SDL_RenderClear(native_renderer_);
}
void Renderer::Present() {
  SDL_RenderPresent(native_renderer_);
}

void Renderer::DrawImage(std::span<const uint32_t> image) {
  int32_t width(800), height(600);
  SDL_GetWindowSize(native_window_, &width, &height);
  SDL_UpdateTexture(texture_, nullptr, image.data(), width * sizeof(uint32_t));
  SDL_RenderTexture(native_renderer_, texture_, nullptr, nullptr);
}

void Renderer::PollEvent() {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_EVENT_QUIT) {
      quit_ = true;
    }
  }
}