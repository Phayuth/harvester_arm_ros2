from rclpy.duration import Duration as rclduration
from builtin_interfaces.msg import Duration as msgduration
from rclpy.time import Time as rcltime
from builtin_interfaces.msg import Time as msgtime

durationa = 12.23
durationb = 12.23
durationc = 23.11

rclda = rclduration(seconds=durationa)
rcldb = rclduration(seconds=durationb)
rcldc = rclduration(seconds=durationc)

msgd = rclda.to_msg()
print(f"> msgd: {msgd}")

a = rclda == rcldb
print(f"> a: {a}")

b = rcldb < rcldc
print(f"> b: {b}")

c = rcldb > rcldc
print(f"> c: {c}")

d = rcldc >= rclda
print(f"> d: {d}")

e = rcldb - rclda
print(f"> e: {e}")

f = rcldc - rclda
print(f"> f: {f}")

g = rcldc + rclda
print(f"> g: {g}")


def check_same_timestamp(da: msgduration, db: msgduration, error=0.01):
    return db - da <= error


h = check_same_timestamp(durationc, durationa)
print(f"> h: {h}")



timea = 12.23
timeb = 12.23
timec = 23.11

rclda = rcltime(seconds=timea).to_msg()
rcldb = rcltime(seconds=timeb).to_msg()
rcldc = rcltime(seconds=timec).to_msg()
print(f"> rcldc: {rcldc}")


a = timea == timeb
print(f"> a: {a}")

b = timeb < timec
print(f"> b: {b}")

c = timeb > timec
print(f"> c: {c}")

d = timec >= timea
print(f"> d: {d}")

e = timeb - timea
print(f"> e: {e}")

f = timec - timea
print(f"> f: {f}")

g = timec + timea
print(f"> g: {g}")