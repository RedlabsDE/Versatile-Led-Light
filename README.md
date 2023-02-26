# Versatile-Led-Light
arduino project for multiple lightning purposes like motion light, night light with timer, ...


## Tag 1.0 - Motion Light battery powered
| State  | Event  | Task  |
|---|---|---|
| Leds Off  | motion detected | fade up + start timer |
| Leds On  | motion detected | re-start timer  |
| Leds On  | timer expired  | fade down and turn off LEDs  |

## Tag 2.0 - Dim and smooth night light controlled by button and timer
| State  | Event  | Task  |
|---|---|---|
| Leds Off  | short button press  | fade up color 1  + start timer "auto-off"|
| Leds Off  | long button press  | fade up color 2  |
| Leds On  | short/long button press  | fade down and turn off LEDs  |
| Leds On  | timer "auto-off" expired | fade down and turn off LEDs  |

## Tag 2.1 - Dim and smooth night light controlled by button
| State  | Event  | Task  |
|---|---|---|
| Leds Off  | short button press  | fade up color 1  |
| Leds Off  | long button press  | fade up color 2  |
| Leds On  | short/long button press  | fade down and turn off LEDs  |
