# ECE 544 Project #2 -- Microprocessor-based Closed Loop Control
## <b>This assignment is worth 100 points.  Demos on Wed, 15-May and Thu 16-May.  Deliverables are due to GitHub and D2L on Fri, 17-May by 10:00 PM.  </b>

### <i> You will do your development using Xilinx Vivado for hardware and SDK for software. You should submit your assignment before the deadline by pushing your final deliverables to your shared GitHub repository for the assignment. For this project we'd also like you to submit an archive (a .zip) file to your Project #2 dropbox on D2L.

#### NOTE:  The use of GitHub and GitHub classroom is required for this project.</i>

## Learning Objectives
- Gain experience with microprocessor-based closed loop control
- Gain experience with multi-tasking, synchronization, and inter-process communication (IPC) using FreeRTOS
- Apply the concepts learned in Project 1 to a more complex application
- Flex your hardware-building muscles w/ “real” circuits


### Introduction

This project is designed to give you practical experience with a common, and often crucial, embedded system function – closed loop control. Closed-loop control involves using feedback from one or more sensors to adjust the input parameters that control the output of the circuit.

The Project #2 control system regulates the speed of a geared motor. The shaft of the motor is connected to a metal hub. You can apply friction to the hub with your finger to change the load on the motor. As the load varies the closed loop control system responds by trying to maintain the drive motor speed. The speed and direction of rotation for the motor are set by a Digilent PmodHB3. The current speed of the motor is reported by the built-in quadrature encoder on the motor.

You can purchase the electronic and electromechanical components you need for this project from the EPL. The total cost of all parts is $52. The BOM for the project is included in the Project 2 release package.

### 	Project teams
You will do this project in teams of 2.  Since both members of the team will receive the same grade it is important to meet with me (Roy) confidentially as early in the project as you can if the partnership isn't working out.  The longer you wait the more difficult it is for us to make corrections.

We will be using the team project support in GitHub classroom for this assignment.  This means that your team will share a private repository on GitHub that can also be accessed by the instructor, Grader, and T/A for the course. You will submit your work via a private repository using GitHub classroom.  

#### Tutorials on using git and GitHub:
- Edureka (1hr, 45 min): https://www.youtube.com/watch?v=xuB1Id2Wxak
- TraversyMedia (32 min): https://www.youtube.com/watch?v=SWYqp7iY_Tc&t=96s

### Tutorials on using version control systems such as Git/GitHub with Vivado
- Xilinx Quicktake video (~ 11 minutes)  https://www.youtube.com/watch?v=L17LvqkAv28
- [Vivado Design Flows Overview](git_vivado/ug892-vivado-design-flows-overview.pdf)
- [Vivado Revision Control Tutorial](git_vivado/ug1198-vivado-revision-control-tutorial.pdf)
- [Xilinx Application Note 1165](git_vivado/xapp1165.pdf)
- [Xilinx Application Note 1165 examples](git_vivado/xapp1165.zip)

### To manage your project under GitHub classroom:
1. Install the Git tools (can be downloaded from https://git-scm.com/) on the PC your are using for the course and create a GitHub account (if you do not already have one)

2. The team member (first) who originally created the Project #2 group in D2L should log into their GitHub account and open the link to the Project #2 assignment in a new browser tab.   This will bring up a screen that allows him/her to either join an existing group or create a new group.

3. That first team member should create a new group and name it like I named it in D2L (ex:  RoyK_MelihH).  You will likely be asked to join the ECE 544 Spring 2019 organization since this is the first assignment that you are using GitHub for.  Follow the instructions (which typically involves clicking on a link in an email you receive) to join the organization).  You should only have to join the organization once for the term.

4. After receiving confirmation that the repository and group was created, the first team member should invite the 2nd team member to join the group.

5. The second team member should log into their GitHub account and paste the assignment link into a new browser tab.  This will bring up a screen that allows you to either join an existing group or create a new group.  This time join your existing group.  As stated above, you will likely be asked to join the ECE 544 Spring 2019 organization if this is the first assignment you accept.

 Please do not join a group that is not your own.  I set a limit of 2 members per team so if you join the "wrong" group before the 2nd member joins the group you will lock out the 2nd member of that group.

6. The second member should get an email w/ their private Group repository for the project. You should both be good to go.

7. Create your project(s) in Vivado.

8. Develop your code, pushing to your private repository to GitHub following the guidelines for using version control with Vivado

9. When you have finished the project, write your design report and include it in your repository along with any other deliverables that aren't in your Vivado project(s).

10. Do a final push of your deliverables to the private repository for the project before the deadline.

## GitHub Deliverables
- A demonstration of your project to the instructor or TA. You must bring your own control and motor system.
- A five to seven (5 to 7) page design report explaining the operation of your design, most notably your control algorithm and user interface. Please include a few “interesting” graphs showing the results from the control algorithm. List the work done by each team member. Be sure to note any work that you borrowed from somebody else.
- Source code for your C application(s). Please take ownership of your application. We want to see your program structure and comments, not ours.
- All source files regarding your new motor control and speed measurement system (IP), including the Verilog hardware, and driver code.
- Your constraint and top level Verilog files.
- A schematic for your embedded system. You can generate this from your block design by right-clicking in the diagram pane and selecting Save as PDF File…

## D2L Deliverables
Submit a .zip file of your repository to your Project #2 dropbox on D2L.  You can create a .zip of your repository by clicking on "Clone or download" in GitHub and selecting "Download ZIP"

## Extra Credit Opportunities
Project 2 offers several opportunities to earn extra credit points. Here are some suggestions but we are willing to be amazed and amused:
- Innovate on the user interface –you want to be able to easily “tinker” with the control loop
- Enhance the tachometer functionality to be able to detect direction of rotation in addition to the speed.
