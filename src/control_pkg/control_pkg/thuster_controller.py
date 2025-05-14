from serial import Serial

serial = Serial('/dev/tty', 115200)


vel1, dir1 = input("ingrese vel1,dir1").strip().split(",")
vel2, dir2 = input("ingrese vel2,dir2").strip().split(",")

rpm_msg = f"{vel1},{vel2},{dir1},{vel2}"

serial.write(rpm_msg.encode())

while True:
    rpm_feedback = serial.readline()

    rpm_motor1, rpm_motor2 = rpm_feedback.strip().split(" | ")

    rpm_motor1 = float(rpm_motor1)
    rpm_motor2 = float(rpm_motor2)

    print(f"Motor 1 = {rpm_motor1} rpm")
    print(f"Motor 2 = {rpm_motor2} rpm")