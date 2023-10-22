# CS350-Emerging-Systems-Architecture-and-Technology
•	**Summarize the project and what problem it was solving.**

This project aimed to implement proper logic and tasks on a task scheduler to create a prototype for a smart Thermostat. The problem within the project was to ensure that each of the recommended tasks was performed at the appropriate time in the timer cycle. Additionally, format the data in a way that can be sent to a server. This was the beginning of a thermostat that used GPIO buttons to adjust the set temperature and the onset of communication with a server to adjust the set temperature from a remote location. 
The project implemented several peripherals installed on the microcontroller, such as GPIO, I2C, and UART, to adjust the set temperature, read the ambient temperature, and relay that information back to the server. 

•	**What did you do particularly well?**

What I did well in this project was to take my time, read all available information, and not panic. My previous milestone work disheartened me, and I was anxious about getting into the programming of this project. After I had a handle on what the available resources (i.e., Zybooks and Lectures) had to offer, I felt more confident approaching the problem and testing my code. 

•	**Where could you improve?**

Previous modules have harped on planning first and then executing, and I thought I was seeing a pattern in some of that code. However, when I omitted the plan, I completely missed some aspects of the pattern. Writing down, even on paper, what I am trying to do with a map is far more helpful than assuming I know what the pattern is. This way, I can check off the handwritten note and see that I have been to each location on that map before testing my code and hoping.

•	**What tools and/or resources are you adding to your support network?**

The tool I used pretty often was from Zybooks and the RIBS State machine builder. That was incredibly helpful in taking my handwritten state machine diagram and turning it into a cleaner-looking diagram with the added benefit of creating the transition and action cases for each state. 

•	**What skills from this project will be particularly transferable to other projects and/or coursework?**

It must be said that looking at the microcontroller at the beginning of the course was slightly intimidating. I have some experience with Raspberry Pi’s. Still, they have an Operating system with multiple languages to interact with. I am more confident in my Python skills than most others. So, Learning C for this course would be a challenge; however, the transition was not as steep as I had first thought because of the foundations of previous courses and their use of C++. Taking that knowledge of C++ and applying it to C was very simple. One project that I know I want to attempt for this TI development board is to have it replace my Raspberry Pi in the garage and have it interface with my Home Assistant so I can get readings from my magnetic pickups to get the doors position and actuate the relays that toggle the door. I think this will be a fun project. Still, it will free up a Raspberry Pi that is underutilized as a garage door monitor.

•	**How did you make this project maintainable, readable, and adaptable?**

I tried harder in this project to ensure each action was in a function with an appropriate name. So, instead of multiple lines of repeated code to turn off or on an LED, creating a function makes the logic more manageable to read in the main or timer flag functions. 
