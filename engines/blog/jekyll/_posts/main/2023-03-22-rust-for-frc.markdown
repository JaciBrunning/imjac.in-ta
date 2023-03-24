---
layout: post
title: "Experiments with Rust for FRC Robots"
date: 2023-03-25 01:30:00
author: Jaci
categories: rust, frc, code
---

Recently I've been playing with a bit of a pet project - writing an experimental FRC robotics framework in Rust. This isn't the first time I've written something ambitious in Rust - my FMS alternative, [JMS](https://github.com/JaciBrunning/JMS), is close to 40,000 lines of it - but as it turns out the language really shines when it comes to robot programming. Buckle up and follow along at [robot.rs](https://github.com/JaciBrunning/robot.rs). <!-- excerpt -->

# Getting Started
Getting started was surprisingly simple, rust's `bindgen` crate has you covered for linking to existing C libraries like WPILib's HAL (Hardware Abstraction Layer). Once you have all the bindings, it's really as simple as just calling them (with a little `hal_safe_call!` macro to handle the unsafe behaviour and error codes):

```rust
impl DigitalRoboRIOInput {
  pub fn new(drr: DigitalRoboRIO) -> Self {
    // Set output direction (1 = input)
    hal_safe_call!(HAL_SetDIODirection(drr.handle, 1)).unwrap();
    Self(drr)
  }
}

impl DigitalInput for DigitalRoboRIOInput {
  fn get(&self) -> bool {
    hal_safe_call!(HAL_GetDIO(self.handle)).unwrap() > 0
  }
}
```

# NetworkTables

NetworkTables was a similar story, with `ntcore` having a set of C bindings for exactly this purpose. With some magic, we can make it incredibly easy to use:
```rust
fn main() {
  let mut instance = NetworkTableInstance::default();
  instance.start_server(Default::default());

  // Write "1234" as an int to /my/entry
  nt!("/my/entry", 1234).unwrap();
  loop {
    // Read back /my/entry
    println!("{:?}", nt!(read "/my/entry"));
    std::thread::sleep(std::time::Duration::from_millis(1000));
  }
}
```

# Composable I/O
Rust, like C++, has the concept of "moving" objects in memory. Essentially, this means stealing an object from one place to another, making only the new one accessible. We can use this to our advantage, by creating composable I/O. 

How many times have you wanted to add a voltage limit to a motor? Or invert a digital output? Well, let's have a look at how simple that can be:
```rust
// Clamp a motor's output to -10V..10V
let motor = ClampedMotor(PWMSparkMax::new(0), -10, 10);

// Create a new digital output, that's inverted.
let out = InvertOutput(DigitalRoboRIO::new(1).output());
```

What's great is that you can use these as a regular motor and digital output since they both implement the desired traits (`MotorController` and `DigitalOutput` respectively). Let's dive into `ClampedMotor` to have a look at how it works:
```rust
pub struct ClampedMotor<M: MotorController>(pub M, pub f64, pub f64);

impl<M: MotorController> MotorController for ClampedMotor<M> {
  fn set_voltage(&mut self, voltage: f64) {
    self.0.set_voltage(voltage.max(self.1).min(self.2))
  }

  fn get_set_voltage(&self) -> f64 {
    self.0.get_set_voltage()
  }
}

wrapped_traits!(MotorController, ClampedMotor);
```

When we create a `ClampedMotor`, we move the motor into the struct. The motor is now under the exclusive ownership of `ClampedMotor`, which allows us to do all kinds of cool things. Foremostly, we don't have to worry about borrow lifetimes since we own it. It also guarantees that all access to the motor is clamped - there is no universe in which the motor violates its minimum and maximum voltage.

The approach that we've taken here is very similar to what embedded firmware in rust does - moving exclusive ownership of peripherals when they become initialised. We also implement `std::ops::Deref` and `std::ops::DerefMut`, which grants you access to all the normal functions of the motor controller - so if you have a smart motor controller (like a Talon SRX or Spark Max), you can still access those functions without ejecting the original motor from the clamp.

# A simple, synchronous example
Let's create a really boring program. All it's going to do is read the left X axis from an Xbox controller and push it out to a motor. 

```rust
pub fn my_robot(running: Arc<AtomicBool>) -> RobotResult {
  let mut motor = PWMSparkMax::new(0);
  let xbox = Xbox::new(0);

  // While the program is running... (Atomic load)
  while running.load(std::sync::atomic::Ordering::Relaxed) {
    // Get the Left X input from the motor and send it to a motor
    let value = xbox.left_x().get();
    motor.set_speed(value);
    std::thread::sleep(std::time::Duration::from_millis(20));
  }
  Ok(())
}

robot_main!(my_robot);
```

Pretty simple, right? We're not even worrying about control mode here - just a basic example. But this is pretty boring, so let's look at something a little more exciting.

# Asynchronous Programming
Using Rust grants us access to its huge ecosystem of crates (libraries), and of course the language's inbuilt support for async programming. Async programming allows you to interleave code that would otherwise run one line after the other - giving you implicit concurrency. Why would we want that? Well, I'll let the below speak for itself.

```rust
async fn auto_routine<'a, E: AbstractElevator<'a>>(elevator: &'a E) -> ElevatorResult<()> {
  info!("Auto Start");
  elevator.go_to_height(0.5).await?;
  elevator.go_to_height(1.0).await?;
  elevator.go_to_height(0.75).await?;
  elevator.go_to_height(0.0).await?;
  elevator.go_to_height(1.0).await?;
  info!("Auto Done");
  Ok(())
}


impl Elevator {
  // ...
  pub async fn go_to_height(&self, height: f64) -> ElevatorResult<()> {
    *self.state.write().await = ElevatorState::HeightControl { height };
    while !self.is_stable().await {
      tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
    }
    Ok(())
  }
  // ...
  pub async fn run(&self) {
    loop {
      let mut demand_voltage = 0.0;
      let current_height = self.config.height_sensor.read().await.get_distance();
      let mut pid = self.pid.write().await;

      match &*self.state.read().await {
        ElevatorState::Idle => pid.reset(),
        ElevatorState::Manual { voltage } => {
          pid.reset();
          demand_voltage = *voltage
        },
        ElevatorState::HeightControl { height } => {
          pid.set_setpoint(*height);
          
          let feedforward = self.config.motor_model.voltage(
            self.config.mass * 9.81 * self.config.spool_radius,
            0.0
          );

          demand_voltage = pid.calculate(current_height, now()).output + feedforward;
        }
      }

      // Publish to NetworkTables, with updated PID coefficients as required
      nt!("elevator/state", format!("{:?}", self.state.read().await)).unwrap();
      nt!("elevator/height", current_height).unwrap();
      nt!("elevator/voltage", demand_voltage).unwrap();
      pid.nt_update("elevator");

      self.config.motor.write().await.set_voltage(demand_voltage);
      tokio::time::sleep(tokio::time::Duration::from_millis(20)).await;
    }
  }
  // ...
}
```

What on earth is going on here? I thought we weren't allowed to use while loops in our robot code, how on earth does this work? Well, that's the magic of async. You can think of each async function as a 'task', with multiple tasks being able to run at the same time. When you `.await` or `join!` a task, you wait for it to complete before moving on.

Because this code can run alongside other code, we can write dead-simple autonomous routines like this without needing to worry about command-based programming or any other structure to manage scheduling. It just happens for us, behind the scenes. Another fringe benefit is that we get to use whatever loop timing we want - you can have your elevator running at 100Hz, your controls at 50Hz, and your drivetrain at 200Hz if you so desire - no need to deal with threads. It just works.

<img src='/ta/img/rust-for-frc/async-elevator-pid.png' style='width:700px;' /><br/>

## Two Cube Auto
Let's have a look at another autonomous example...

```rust
async fn two_piece_auto(elevator: Arc<Elevator>, drivetrain: Arc<Drivetrain>, gripper: Arc<Gripper>) {
  info!("Two Cube Auto Begin");
  // Drop the first gamepiece
  gripper.release().await;
  info!("That's Cube One!");

  // Turn 180
  drivetrain.drive_distance(-0.1).await;
  drivetrain.turn_angle(180.0).await;

  // Run at the same time - drop elevator to 0.2 and drive to the setpoint
  let elev = elevator.go_to_height(0.2);
  let drive = async {
    drivetrain.drive_distance(0.5).await;
    drivetrain.turn_angle(90.0).await;
    drivetrain.drive_distance(0.2).await;
  };
  join!(elev, drive);

  // Grab the gamepiece
  gripper.grip().await;
  info!("Grabbed Cube Two");

  // Raise the elevator and back off
  let elev = elevator.go_to_height(1.0);
  let drive = drivetrain.drive_distance(-0.5);
  join!(elev, drive);

  // Turn back towards the scoring area, drive over
  drivetrain.turn_angle(90.0).await;
  drivetrain.drive_distance(0.5).await;

  // Drop the 2nd gamepiece
  gripper.release().await;
  info!("That's Cube Two!");
  info!("Two Cube Auto Stop");
}
```

Isn't that just pleasant? Assuming your subsystems implement the right traits, you can do a really complex auto in only a few dozen lines of code that just seems to flow like a nice, calm river. It's almost like an art form.

## Asynchronous internals and async on the RoboRIO
The way async works under the hood is by setting up a threadpool, usually up to the total number of cores on your system. Then, as your code runs, each time you comes across an `await` or `join`, that piece of work gets suspended and another piece of work spins up. When that piece of work hits an `await` or `join`, another piece of work. And so on, and so on, for as much work as there exists. Essentially, your control logic gets interlaced, allowing the CPU to spend a little bit of time on each section. This does come at a bit of a performance cost due to context switching, but it also grants you pseudo-concurrency with an incredible amount of flexibility. The runtime can also balance this work across CPU threads, meaning you don't have to worry about threads for the most part.

I'm yet to see how this plays out on a RoboRIO - I've only done it so far on my day-to-day system in simulation. Though, I imagine it would be more than performant enough for most usecases. I'd really love to put it through its paces. Perhaps an experiment for the offseason. 

# Control Locks
Only three things are certain in life - death, taxes, and dammit the drivetrain's fighting with itself again. With so many things going on in a robot at once - manual control combined with autonomous actions, sensor-driven superstructures that are triggered by the codriver, etc - it's easy to see how sometimes you can be commanding the same subsystem from two parts of the code at the same time, with each fighting for control of what must by its nature be an exclusive resource.

In WPILib, we solve this problem with the command-based `Requires()` method. With asynchronous programming, our approach is a little different. Some may be quick to jump at the idea of a Mutex or RwLock, both of which grant exclusive write access to a resource, but these both have a flaw that make them inapplicable to our use case - the owner has to willingly give up control. If we're waiting for the owner to finish it's task, it makes it impossible for us to interrupt it without a bunch of extra work. Why would we do this? Well, if your elevator is under manual control and you want to make it perform an autonomous action like go to a height, we need to interrupt that manual control.

This is where the idea of Control Locks come in. I originally architected these back in 2015, then they went dormant for half a decade, and now they're back! Control Locks give you exclusive control over a resource, but they allow that exclusive control to be 'stolen' by another piece of code if required. Let's have a look at an example from our elevator:

```rust
async fn auto_routine<'a, E: AbstractElevator<'a>>(elevator: &'a E) -> ElevatorResult<()> {
  info!("Auto Start");
  let control = elevator.control().steal().await;

  // Note the `.await?`. The question-mark operator allows the auto routine to bail early
  // if the control lock gets stolen, or the elevator otherwise runs into an error.
  elevator.go_to_height(0.5, &control).await?;
  elevator.go_to_height(1.0, &control).await?;
  elevator.go_to_height(0.75, &control).await?;
  elevator.go_to_height(0.0, &control).await?;
  elevator.go_to_height(1.0, &control).await?;

  Ok(())
}

async fn go_to_height(&self, height: f64, lock: &ControlLock<'a, Self>) -> ElevatorResult<()> {
  while let Some(_) = lock.get().await {
    *self.state.write().await = ElevatorState::HeightControl { height };
    if self.is_stable().await {
      return Ok(())
    }
    tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
  }
  Err(ElevatorError::Interrupted)
}
```

Our auto routine 'steals' access to the elevator, interrupting whoever was using it last (such as manual control). The elevator then passes the control lock to each subcommand, each of which checks that we have access to the lock, otherwise returning an error. The errors are propagated with `?`, which is similar to a rethrow in C++ or Java. That way, if we get interrupted half-way through (by pressing a button to take back manual control, for example), the elevator doesn't continue to try and go to setpoints. 

Ideally, we have one control lock for each subsystem we want to control. You can think of a control lock as a license to use a specific system. You need to show the bouncer your ID before you walk in.

# Button Input
Let's have a look at a simple example of using buttons to trigger asynchronous code:
```rust
/// Simple function to handle button presses scheduling new activities
async fn buttons<'a, E: AbstractElevator<'a> + Send + Sync>(elevator: &'a E, xbox: &Xbox) {
  let mut futs = vec![];
  futs.push(xbox.a().edge_take(Edge::Falling).run(|| { move_to_height(elevator, 1.0) }).boxed());
  futs.push(xbox.b().edge_take(Edge::Falling).run(|| { manual_controls(elevator, xbox) }).boxed());

  // Join all the futures, running them concurrently within this function until either of them complete (never)
  join_all(futs).await;
}
```

This can look a little dense at the start, but it's actually remarkably simple. The key component looks like the following:
```rust
xbox.a().edge_take(Edge::Falling).run(|| { move_to_height(elevator, 1.0) })
```

On the falling edge of the A button on the xbox controller, move the elevator to the height 1.0m. 

# Moving Forward
So what now? Well, I'd love to flesh out the framework and getting it running on a real robot to see how it performs. I think for the moment it'll remain experimental, but if there's anyone who's brave enough to put it on a robot (even for offseason) - well, you know where to find me. 
