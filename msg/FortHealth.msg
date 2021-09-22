uint8 battery_level               # 1-100%, in 10% increments
bool battery_charging             # 0/1 "bool"
uint8 connection_strength_vsc     # low 0/1/2 high : as percieved by VSC
uint8 connection_strength_src     # low 0/1/2 high : as percieved by SRC
uint8 rssi_vsc                    # documentation says "Reserved for future use"
uint8 rssi_src                    # documentation says "Reserved for future use"
uint8 vsc_mode  # 0 = Error
                # 4 = Local/Searching
                # 6 = Remote Connected
                # 9 = Remote Operational
                # 10 = Menu
                # 11 = Pause
