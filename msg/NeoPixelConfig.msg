# Datastream frequencies
uint8 NEO_KHZ800 = 0 # 0x0000
uint8 NEO_KHZ400 = 1 # 0x0100

# Pixel color order
uint8 NEO_BGR = 164
uint8 NEO_BRG = 88
uint8 NEO_RGB = 6
uint8 NEO_GRB = 82
uint8 NEO_GBR = 161
uint8 NEO_RBG = 9


uint8 stripe_length # Length of the NeoPixel stripe
uint8 pixel_freq    # Pixel datastream frequency. Either NEO_KHZ400 or NEO_KHZ800
uint8 color_order   # Color order: Any value defined by Adafruit NeoPixel Arduino library
uint8 pin_number    # Pin number connected to Data In (DI) pin of NeoPixel
