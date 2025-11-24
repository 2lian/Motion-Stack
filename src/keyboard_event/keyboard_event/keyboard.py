import asyncio
import colorsys
import dataclasses
import os
import threading
from typing import Any, Callable, Dict, KeysView, List, Self, Tuple

import asyncio_for_robotics.ros2 as afor
import rclpy
import sdl2
import sdl2.ext
from ament_index_python.packages import get_package_share_directory
from asyncio_for_robotics.core.sub import BaseSub
from keyboard_event_msgs.msg import Key as KeyMsg
from rclpy.node import Node


class TOPICS:
    ALL = afor.TopicInfo("keyall", KeyMsg)
    DOWN = afor.TopicInfo("keydown", KeyMsg)
    UP = afor.TopicInfo("keyup", KeyMsg)


def scancode_to_color(scancode):
    if scancode == 0:
        return 255, 0, 0
    brightness = 255
    r, g, b = colorsys.hsv_to_rgb(
        (scancode % 30) / 30, 0.5, 1
    )  # HSV: full saturation, full value

    return int(r * brightness), int(g * brightness), int(b * brightness)


@dataclasses.dataclass(frozen=True)
class Key:
    symbol: bytes
    code: int
    modifiers: int
    is_pressed: bool
    sdl_event: sdl2.SDL_KeyboardEvent

    @classmethod
    def from_sdl(cls, sdl_event: sdl2.SDL_KeyboardEvent) -> Self:
        return cls(
            symbol=sdl2.SDL_GetKeyName(sdl_event.keysym.sym),
            code=sdl_event.keysym.scancode,
            modifiers=sdl_event.keysym.mod,
            is_pressed=bool(sdl_event.state),
            sdl_event=sdl_event,
        )


def sdl_thread(
    asyncio_loop: asyncio.AbstractEventLoop,
    stop_event: threading.Event,
    sub_input: Callable[[Key], None] = lambda *_: None,
):
    sdl2.ext.init()
    window = sdl2.ext.Window("Input", size=(100, 100))

    pkg_share = get_package_share_directory("keyboard_event")
    surface_gogo_calm = sdl2.ext.load_img(
        os.path.join(pkg_share, "icons", "gogo.bmp")
    )
    surface_gogo_happy = sdl2.ext.load_img(
        os.path.join(pkg_share, "icons", "gogo_happy.bmp")
    )
    surface_gogo_happy2 = sdl2.ext.load_img(
        os.path.join(pkg_share, "icons", "gogo_happy2.bmp")
    )
    sdl2.SDL_SetWindowIcon(window.window, surface_gogo_calm)
    window.show()

    renderer = sdl2.ext.Renderer(window)
    texture_gogo_calm = sdl2.SDL_CreateTextureFromSurface(
        renderer.sdlrenderer, surface_gogo_calm
    )
    texture_gogo_happy = sdl2.SDL_CreateTextureFromSurface(
        renderer.sdlrenderer, surface_gogo_happy
    )
    texture_gogo_happy2 = sdl2.SDL_CreateTextureFromSurface(
        renderer.sdlrenderer, surface_gogo_happy2
    )
    sdl2.SDL_FreeSurface(surface_gogo_calm)
    sdl2.SDL_FreeSurface(surface_gogo_happy)
    sdl2.SDL_FreeSurface(surface_gogo_happy2)
    dst_rect = sdl2.SDL_Rect(0, 0, 100, 100)  # x, y, width, height

    renderer.color = scancode_to_color(0)
    renderer.clear()
    sdl2.SDL_RenderCopy(renderer.sdlrenderer, texture_gogo_calm, None, dst_rect)
    renderer.present()
    window.refresh()

    running = True
    pressed: Dict[int, Key] = dict()
    cycle = 0
    texture_cycle = [texture_gogo_happy, texture_gogo_happy2]
    while running:
        events = sdl2.ext.get_events()
        if stop_event.is_set():
            return 0

        for e in events:
            if e.type == sdl2.SDL_QUIT:

                def finish():
                    raise KeyboardInterrupt

                asyncio_loop.call_soon_threadsafe(finish)
                return 0

            if stop_event.is_set():
                return 0

            elif e.type == sdl2.SDL_KEYDOWN:
                if e.key.repeat:
                    continue
                k = Key.from_sdl(e.key)
                asyncio_loop.call_soon_threadsafe(sub_input, k)
                pressed[k.code] = k

            elif e.type == sdl2.SDL_KEYUP:
                k = Key.from_sdl(e.key)
                asyncio_loop.call_soon_threadsafe(sub_input, k)
                del pressed[k.code]
            else:
                continue

            renderer.color = (
                scancode_to_color(list(pressed.keys())[-1])
                if len(pressed) > 0
                else scancode_to_color(0)
            )
            renderer.clear()
            if len(pressed) > 0:
                cycle = (cycle + 1)%len(texture_cycle)
                sdl2.SDL_RenderCopy(
                    renderer.sdlrenderer, texture_cycle[cycle], None, dst_rect
                )
            else:
                sdl2.SDL_RenderCopy(
                    renderer.sdlrenderer, texture_gogo_calm, None, dst_rect
                )
            renderer.present()

        window.refresh()
    return 0


async def async_sdl(
    down_callback: Callable[[Key], Any] = lambda *_: None,
):
    stop_event = threading.Event()
    sdl_coro = asyncio.to_thread(
        sdl_thread, asyncio.get_event_loop(), stop_event, down_callback
    )
    try:
        await sdl_coro
    except:
        stop_event.set()
        sdl2.ext.quit()


class KeySub(BaseSub[Key]):
    pass


def start() -> Tuple[KeySub, asyncio.Task]:
    sub = KeySub()
    sdl_task = asyncio.create_task(async_sdl(sub._input_data_asyncio))
    return sub, sdl_task


def key_to_ros(k: Key) -> KeyMsg:
    msg = KeyMsg()
    msg.symbol = k.symbol.decode()
    msg.code = k.code
    msg.modifiers = k.modifiers
    msg.is_pressed = k.is_pressed
    return msg


async def async_main():
    key_sub, sdl_task = start()
    try:
        with afor.auto_session().lock() as node:
            pub_all = node.create_publisher(*TOPICS.ALL.as_arg())
            pub_down = node.create_publisher(*TOPICS.DOWN.as_arg())
            pub_up = node.create_publisher(*TOPICS.UP.as_arg())
            clock = node.get_clock()
        async for k in key_sub.listen_reliable():
            msg = key_to_ros(k)
            msg.header.stamp = clock.now().to_msg()
            pub_all.publish(msg)
            if k.is_pressed:
                pub_down.publish(msg)
            else:
                pub_up.publish(msg)
    finally:
        sdl_task.cancel()
        await sdl_task


def main():
    rclpy.init()
    ses = afor.ThreadedSession(node=Node(node_name="keyboard_event"))
    afor.set_auto_session(ses)
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass
    finally:
        sdl2.ext.quit()
        ses.close()
        rclpy.shutdown()
    print("Exited cleanly :)")


if __name__ == "__main__":
    main()
