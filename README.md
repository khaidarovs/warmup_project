# warmup_project

## Driving in a square

### High-level description

I approached this task as being a modification of the spin_in_circles code. I wanted to make the turtlebot move in the x-direction using Twist and /cmd_vel for a certain amount of time, then make a 90 degree turn and repeat the sequence 4 times. 

### Code explanation

I took the spin_in_circles code as a basis for my code. I initialized the node to "drive_square", and the publisher to publish Twist messages to the /cmd_vel topic. The run() function consists of two while loops, one for travelling straight, and the other one for turning. The inner loop runs while current_distance <= distance (4 in my code), and sleeps for 1 second in the process before publishing the message. This means that the turtlebot will drive straight for roughly 4 seconds, at x-axis velocity of 0.2. After the inner loop ends, the next message being published stops the turtlebot, and sets the angular z-velocity to 0.7854. The reason it is 0.7854 is because 0.7854*2 is 1.5708 or roughly 90 degrees in radians. If we set the turning time to 2 seconds, the turtlebot should therefore turn roughly 90 degrees. In my case, through trial and error, I found that setting the turning time (rospy.sleep()) to 2.2 seconds gave a sharper 90 degree turn. After the 2.2 seconds end, the next Twist message stops the robot, we increment the current_turn variable, and the outer loop starts again. This continues until the turtlebot has completed 4 turns. After that (after the outer loop is done running), the last message stops the turtlebot.

### Gif

https://user-images.githubusercontent.com/93730296/161688333-997192b8-a8ff-44e5-a40e-f3097888beca.mp4


### Challenges

The main challenge was to find a way for the turtlebot to travel for a specific period of time and to make it turn exactly 90 degrees. In order to address the first challenge, I used the rospy.sleep() function that allowed me to do a specific action on the turtlebot for a certain time. For example, it allowed me to move the turtlebot forward for 1 second 4 times, equalling 4 seconds in total, and it also allowed me to rotate the robot for roughly two seconds with an angular speed of 0.7854, to achieve roughly a 90 degree turn. Adding on to the challenge of making the turtlebot turn 90 degrees, I had to utilize the function which states that angular velocity = change in theta (radians) / change in time. This allowed me to calculate the necessary angular velocity and time for rotation needed for a 90 degree turn.

### Future work

If I had more time, I would experiment more with the angle of rotation. In a few cases, the turtlebot would not turn 90 degrees, but would turn a bit more or less, resulting in non-square movement. I would also want to see how different surfaces affect how fast the robot is turning - I noticed that when one of its wheels was on the yellow tape in the lab, it made a turn of more than 90 degress, perhaps due to less friction than on the lab floor. 

### Takeaways

- My first takeaway is the general framework of how the turtlebot works. I learned about the sending messages to a specific topic functionality and how to make the robot move in the simplest ways using the /cmd_vel topic. I also had a chance to play around with physical turtlebots and learned how to interact with them, which would definitely be very useful for future projects.
- Another takeaway is learning how to write code in an object-oriented way, initializing the nodes and publishes, and writing methods. This is definitely a cleaner way to write code and will be extremely useful in future projects. 
