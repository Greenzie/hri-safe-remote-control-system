uint8 src_battery_level           # 1-100%, in 10% increments
bool  src_battery_charging        # 0/1 "bool"
uint8 vsc_connection_strength     # low 0/1/2 high : as percieved by VSC
uint8 src_connection_strength     # low 0/1/2 high : as percieved by SRC
uint8 vsc_rssi                    # documentation says "Reserved for future use"
uint8 src_rssi                    # documentation says "Reserved for future use"
uint8 vsc_mode  # 0 = Error
                # 4 = Local/Searching
                # 6 = Remote Connected
                # 9 = Remote Operational
                # 10 = Menu
                # 11 = Pause
