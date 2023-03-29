import board
import neopixel

pixels = neopixel.NeoPixel(board.D18, 5, pixel_order=neopixel.GRB)

pixels.fill((0, 255, 0))