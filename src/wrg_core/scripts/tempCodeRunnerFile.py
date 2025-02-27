request = SetBool.Request()
    request.data = True

    future = servo.call_async(request)

    rclpy.spin_until_future_complete(node, future)