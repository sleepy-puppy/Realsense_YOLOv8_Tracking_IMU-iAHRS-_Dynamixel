from serial import Serial
from pyubx2 import UBXReader
 
# amend Serial parameters for your particular device
with Serial("/dev/ttyACM0", 9600, timeout=3) as stream:
    ubr = UBXReader(stream)
    for raw, parsed in ubr:
        # not all messages contain lat and lon values
        if hasattr(parsed, "lat") and hasattr(parsed, "lon"):
            latitude = parsed.lat
            longitude = parsed.lon
            #print(f"{parsed.identity}: lat: {parsed.lat}, lon: {parsed.lon}")
            print(latitude, ',', longitude)