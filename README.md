# quadcopter_simulation
This is a C++ Real-Time Quadcopter Simulation. It calculates the flight of the quadcopter by solving its differential equation. The quadcopter includes four noisy sensors: accelerometer, gyroscope, magnetometer and barometer. The frame mode is X, but it can be changed to +. The sensor fusion uses a 1st order complementary filter, the angle stabilizer consist of two concatenated PID controllers, likewise the altitude control. The sensors are currently called 200 times per second, the user input is taken from the keyboard and updated 50 times per second.

The entire simulation consist of about 2000 lines of code, so it's still overseeable.

## How does it look like?

[YouTube Video] (https://www.youtube.com/watch?v=DGXSOvx3GmY)

## What is it good for?

I used it to build a quadcopter of 450 mm and 1.2 kg. I can copy the stabilization algorithms for roll, pitch and yaw one-to-one (apart from some minor C++ related changes) and the quadcopter flies quite well. It also helped me to understand what impact noise can have, how the motors work, why you need accelerometer *and* gyroscope etc.

And I believe it saved me a lot of money.

## How to compile?

- Download the files.  
- Open the project in Visual Studio C++ (I use Express 2015).  
- Two libraries have to be linked:  
  - [Eigen] (http://eigen.tuxfamily.org/index.php?title=Main_Page) for linear algebra  
  - [Irrlicht](http://irrlicht.sourceforge.net/) for rendering  
- Now it should compile.
- When running the .exe files make sure the folder 'media' with its content and the Irrlicht.dll are found. Eigen doesn't have a .dll file.

## Can it be used with Linux?

It the current state, no. But if anybody can me show a simple Linux equivalent to Windows' 'GetAsyncKeyState()' I would be happy to port it.
