# import sys
# sys.path.append('D:\\vs2019ws\PythonCtt\lib')
import time
import jkrc

robot = jkrc.RC("10.5.5.100")#返回一个机器人对象
robot.login()#登录

# robot.power_on()#上电
# robot.enable_robot()
# Enable = True
# robot.servo_move_enable(Enable)
start_time = time.time()#记录开始时间
ret = robot.get_digital_output(0,0)
if ret[0] == 0:
    print("1the DO2 is :", ret[1])
else:
    print("some things happend,the errcode is: ", ret[0])
ret1 = robot.get_tcp_position()
ret2 = robot.get_joint_position()
# ret = robot.set_digital_output(0, 0, 1)
# ret = robot.set_digital_output(0, 0, 0)

# time.sleep(5)
# ret = robot.set_digital_output(0, 2, 1)
print(ret1)
print(ret2)

end_time = time.time()#记录结束时间
elapsed_time = end_time - start_time  # 计算耗时
print(f"Time taken to get joint position: {elapsed_time:.6f} seconds")
# if ret[0] == 0:
#     print("the joint position is :",ret[1])
# else:
#     print("some things happend,the errcode is: ",ret[0])


# import sys
# import time
# import jkrc
#
# # Initialize the robot object
# robot = jkrc.RC("10.5.5.100")
# ret = robot.login()  # Login to the robot
# if ret[0] != 0:
#     print("Login failed with error code:", ret[0])
#     sys.exit(1)
# # Enable = True
# # robot.servo_move_enable(Enable)
# # Parameters for frequency measurement
# test_duration = 5.0  # Duration to measure frequency (in seconds)
# call_count = 0  # Counter for the number of calls
# start_time = time.time()  # Record start time
#
# # Main loop to call get_joint_position and measure frequency
# while (time.time() - start_time) < test_duration:
#     ret = robot.get_joint_position()
#     if ret[0] == 0:
#         print("Joint position:", ret[1])
#         call_count += 1  # Increment call counter
#     else:
#         print("Error occurred, error code:", ret[0])
#         break  # Exit on error
#
# # Calculate and print frequency
# elapsed_time = time.time() - start_time
# frequency = call_count / elapsed_time
# print(f"\nTest duration: {elapsed_time:.2f} seconds")
# print(f"Number of calls: {call_count}")
# print(f"Frequency: {frequency:.2f} Hz")

