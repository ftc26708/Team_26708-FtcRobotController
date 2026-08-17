# Team 26708 FTC Robot Controller

Welcome to the official repository for **FTC Team 26708**. This project contains the source code for our competition robot, utilizing the FTC SDK along with advanced libraries like **Pedro Pathing** and **Limelight Vision**.

---

## 🏗 Project Architecture

We follow a **Centralized Hardware Pattern** to keep our code clean, maintainable, and reusable across multiple OpModes.

### Core Structure
- **`:TeamCode`**: All team-specific logic resides here.
    - **`org.firstinspires.ftc.teamcode.hardware`**: Contains the `Robot` class and specific subsystem classes (Shooter, Intake, Drivetrain).
    - **`org.firstinspires.ftc.teamcode.opmodes`**: TeleOp and Autonomous implementations.
    - **`org.firstinspires.ftc.teamcode.pedroPathing`**: Configuration and tuning for the Pedro Pathing follower.

### The `Robot` Class
The [Robot.java](file:///C:/Users/danie/Downloads/Team_26708-FtcRobotController-master%20(1)/Team_26708-FtcRobotController-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/hardware/Robot.java) class acts as the single entry point for all robot functions. Instead of OpModes accessing hardware directly, they call methods on the `Robot` instance.

> [!IMPORTANT]
> **Always add new hardware to the `Robot` class first.** This ensures the subsystem is properly initialized and can be used by both Auto and TeleOp.

---

## 🛠 Development Workflows

### How to Add New Hardware
1.  **Create/Update Subsystem**: If it's a new system (e.g., a Climber), create a new class in the `hardware` package.
2.  **Declare in `Robot`**: Add the motor/servo declaration and initialization in the `Robot` constructor.
3.  **Expose Methods**: Create high-level methods in `Robot.java` (e.g., `robot.startIntake()`).

### How to Implement a New Feature
1.  **Define the Logic**: Add the core math or sequence in `Robot.java`.
2.  **Map Controls**: Open [DecodeTeleOp.java](file:///C:/Users/danie/Downloads/Team_26708-FtcRobotController-master%20(1)/Team_26708-FtcRobotController-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/DecodeTeleOp.java).
3.  **Update Logic Loops**: Add your button checks to `drivetrainLogic()` or `mechanismLogic()`.

---

## 💾 State Management (`DataPasser`)

We use a static internal class `Robot.DataPasser` to share critical state between OpModes:
- **Alliance**: Set during initialization in TeleOp based on Auto data.
- **Starting Pose**: Passes the robot's final coordinate from Auto to TeleOp so the drivetrain knows where it is.
- **Timers**: Global action timers for coordinated movements.

---

## 🚀 Key Libraries & Tools

### Pedro Pathing
Used for precise, curved autonomous movement.
- Configuration: [Constants.java](file:///C:/Users/danie/Downloads/Team_26708-FtcRobotController-master%20(1)/Team_26708-FtcRobotController-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java)
- Tuning: [Tuning.java](file:///C:/Users/danie/Downloads/Team_26708-FtcRobotController-master%20(1)/Team_26708-FtcRobotController-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Tuning.java)

### Limelight Vision
Used for relocalization and auto-aiming.
- The `Localization.java` class handles camera-to-field math.
- Use `robot.relocalizeOdoWithCamera()` to correct odometry drift.

---

## 🤝 Collaborative Coding Standards

- **Use Descriptive Names**: `intakeMotor` is better than `motor1`.
- **Telemetry First**: Always add telemetry for new features to help with debugging on the field.
- **Safety**: Use `robot.breakPathFollowing()` when switching to manual control to prevent conflicts.

---

### Setup Guide
If you are new to the team:
1.  Install **Android Studio Ladybug (2024.2)** or later.
2.  Clone this repository.
3.  Open the project and wait for Gradle to sync.
4.  Happy coding! 🤖
