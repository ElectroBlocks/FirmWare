# Arduino Python Firmware

## LED Matrix 
register::ma::10::11::12::1|
write::ma::12::1::1::1::1|
write::ma::12::2::B10011000::B00011000::B00011000::B00011000::B00011000::B00011000::B00011000::B00011000|

## Passive Buzzer 
register::bu::9|
write::bu::9::123|
write::bu::9::0|

## Servos
register::ser::9|
register::ser::11|
write::ser::9::100|
write::ser::11::200|


## LCD

register::lcd::A5::4::20::63|
write::lcd::A5::1| // clear
write::lcd::A5::3| // backlight off
write::lcd::A5::2| // backlight on
write::lcd::A5::5::3::5| //blink 4th row and 6th column zero index based
write::lcd::A5::4| turn off blink
write::lcd::A5::6| scroll right
write::lcd::A5::7| scroll left
write::lcd::A5::8:: HELLO ::W ORL D::Another::    Test…| print on all rows
write::lcd::A5::9::0::7::World| print on first row and 8 column
 

## NeoPixels
register::leds::A0::30::81::10| // 10 is the brightness / 81 is 800 hz and the 1 is GRB / A0 is the pin
write::leds::A0::2::FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00FF000000FF00| //alternating colors
write::leds::A0::1| show whatis there
write::leds::A0::3::5::FF00FF| Set the 5th led to a purple color

## DHT
register::dht::6::1| register a DHT11 sensor use 2 for DHT22
sense| to get reading