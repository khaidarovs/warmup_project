# warmup_project

## Driving in a square

### High-level description

I approached this task as being a modification of the spin_in_circles code. I wanted to make the turtlebot move in the x-direction using Twist and /cmd_vel for a certain amount of time, then make a 90 degree turn and repeat the sequence 4 times. 

### Code explanation

I took the spin_in_circles code as a basis for my code. I initialized the node to "drive_square", and the publisher to publish Twist messages to the /cmd_vel topic. The run() function consists of two while loops, one for travelling straight, and the other one for turning. The inner loop runs while current_distance <= distance (4 in my code), and sleeps for 1 second in the process before publishing the message. This means that the turtlebot will drive straight for roughly 4 seconds, at x-axis velocity of 0.2. After the inner loop ends, the next message being published stops the turtlebot, and sets the angular z-velocity to 0.7854. The reason it is 0.7854 is because 0.7854*2 is 1.5708 or roughly 90 degrees in radians. If we set the turning time to 2 seconds, the turtlebot should therefore turn roughly 90 degrees. In my case, through trial and error, I found that setting the turning time (rospy.sleep()) to 2.2 seconds gave a sharper 90 degree turn. After the 2.2 seconds end, the next Twist message stops the robot, we increment the current_turn variable, and the outer loop starts again. This continues until the turtlebot has completed 4 turns. After that (after the outer loop is done running), the last message stops the turtlebot.

### Gif

https://user-images.githubusercontent.com/93730296/161688333-997192b8-a8ff-44e5-a40e-f3097888beca.mp4


## Person Follower

### High-level description

I approached this task as being a modification of the line_follower code from Lab B. The idea was to determine the angle and direction in which the person was standing, set angular velocity to turn in that direction and keep the turtlebot moving by setting x-linear velocity. 

### Code explanation

I took the line_follower code as a basis for my code. I initialized the node to "follow_person", initialized the subscriber to the /scan topic and the publisher to publish Twist messages to the /cmd_vel topic. The process_scan() callback function analyzes the direction in which the robot should be moving, sets the angular and linear velocities and publishes the commands. The idea was to find the angle at which the distance reported by the scan sensor is within the acceptable range (within 2 metres from the robot). Then depending on whether this angle was more or less than 180 degrees, the robot would have negative and positive angular velocities respectively, with magnitude being proportional to how many degrees the robot should turn. The run() function just kept the program running.

### Gif


https://user-images.githubusercontent.com/93730296/162892061-b7ec4cd9-093f-4edf-b5e8-a3c029195faa.mov


## Wall Follower

### High-level description

I approached this task in a similar way to person follower. The idea was to first find a wall, move to it, and then start following it, adjusting the course of movement along the way, depending on where exactly the wall was relative to the turtlebot. 

### Code explanation

I took the line_follower and person_follower codes as a basis for my code. I initialized the node to "follow_wall", initialized the subscriber to the /scan topic and the publisher to publish Twist messages to the /cmd_vel topic. The process_scan() callback function analyzes the direction in which the robot should be moving, sets the angular and linear velocities and publishes the commands. The idea was to find the minimum non-zero distance to the nearest wall, the direction of the wall and start moving towards it. This happens in the first if-branch of my code and is very similar to the person-follower code. Once the robot is within the acceptable range of the wall, it starts correcting its course based on how it is positioned relative to the wall. If the angle at which the minimum distance to the wall is recorded, is less than 90 degrees (robot is turned towards the wall) the angular velocity is negative so that it would turn right away from the wall. The same logic applies when the angle is more than 90 degrees. The run() function just kept the program running.

### Gif

https://user-images.githubusercontent.com/93730296/162893959-b71ceebe-6593-4e28-9a6d-59be604c9411.mp4

## Challenges

The main challenge of the project was to get used to the controls and how we should interact with the turtlebot. More specifically, I found the adaptive behaviour that we had to deal with very challenging to program. When the turtlebot has to alter its behavior (speed, anglular velocity, etc.) based on the information it receives from the environment, it has quite hard at first to understand how to exactly program it. The challenge for was that I was thinking sequentially, meaning that I tried to program every single behavior (move forward, turn, etc.) that the turtlebot had to perform, instead of making the behaviour adaptive and autonomous. The last two parts of the project really helped me to understand how to work with this kind of adaptive behavior and in my case I used the angles and distances reported by the scan topic to alter the direction of movement of the robot accordingly.

## Future work

If I had more time, I would definitely experiment with making the rotations of the turtlebot more smooth, which would apply to all three parts of this project. In the case of the person_follower and wall_follower, I would definitely account more for the noise in the environment (mistaking a chair for a person, etc.) and try to make the code more robust to instances such as people walking around while the robot is trying to follow me. I would also want to see how different surfaces affect how fast the robot is turning - I noticed that when one of its wheels was on the yellow tape in the lab, it made a turn of more than 90 degress, perhaps due to less friction than on the lab floor.

## Takeaways

- My first takeaway is the general framework of how the turtlebot works. I learned about the sending messages to a specific topic functionality and how to make the robot move in the simplest ways using the /cmd_vel topic. I also had a chance to play around with physical turtlebots and learned how to interact with them, which would definitely be very useful for future projects.
- Another takeaway is learning how to write code in an object-oriented way, initializing the nodes and publishers, and writing methods. This is definitely a cleaner way to write code and will be extremely useful in future projects. 
