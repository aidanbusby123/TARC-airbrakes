import pandas as pd
import math

def getairdensity(air_pressure, air_temp):
     return (2.9 * air_pressure / (8.31432 * (air_temp + 273.15)))


def calc_cd(row):
    if row["vy_local"] > 0:
            return math.sqrt((2 * row["msg_type"]/1000 * abs(-9.8 -row["ay_local"])) / (getairdensity(row["altitude"], row["baro_pressure"]) * 0.00343 * row["vy_local"] * row["vy_local"]))


csv = pd.read_csv('/media/aidan/46C0-DB9D/0.csv')

csv['cd'] = csv.apply(calc_cd, axis=1)

print(csv)
csv.to_csv('test.csv', index=False)