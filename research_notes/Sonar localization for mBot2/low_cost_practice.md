# Localization in practice on low-cost robots with poor sensors (context: mBot2, one HC-SR04-class sonar, encoders, integer gyro, 5 cm grid, ~80-reading 360° sweeps in ~9 s)

## 1. Robot vacuums before cameras/lidar: how iRobot, Evolution/Mint and Neato handled localization

### Takeaway
The first Roombas did not localize at all. They covered rooms with reactive behaviors (random bounce, wall following, spiral) driven by bumpers and a side IR wall sensor. Later non-camera iRobot designs used gyro plus encoders for dead reckoning and corrected drift by re-encountering wall-segment and corner "landmarks" found during wall following, under a rectilinear-room assumption. Products that needed real position either added an external beacon (Mint/Braava NorthStar ceiling spots), a cheap purpose-built 360° laser (Neato, under $30 BOM), or a camera with VSLAM (Roomba 980). None of them used a single swept sonar.

### Cited Findings
- Early Roombas ("iAdapt 1.0") used "random bounce" driven by bump sensors, plus wall following with an edge sensor along the room perimeter. — [explainthatstuff](https://www.explainthatstuff.com/how-roomba-works.html); [The Zebra](https://www.thezebra.com/resources/home/how-roomba-works/)
- iRobot patent US 7,173,391 covers multi-mode coverage (combining bounce, wall following, spot modes), and US 6,690,134 covers localization and confinement (virtual-wall beacons). This is behavior-based coverage, not pose estimation. — [explainthatstuff summary of patents](https://www.explainthatstuff.com/how-roomba-works.html)
- IEEE Spectrum on the Roomba 980 (2015): earlier Roombas used pseudorandom navigation, kept no persistent map, and could not resume where they left off after recharging. The 980 added a ~45° upward camera for VSLAM (tracking corners and furniture features), a new downward odometry sensor, and gyro/IMU. iRobot CEO Colin Angle said the camera costs about $0.75 and VSLAM runs on custom DSPs rather than an expensive processor. He also said "it's very difficult to create a real usable map of the environment." (These are vendor statements, not measurements.) — [IEEE Spectrum](https://spectrum.ieee.org/irobot-brings-visual-mapping-and-navigation-to-the-roomba-980)
- iRobot wall-following patents describe a 3-axis gyro plus 3-axis accelerometer plus wheel encoders, fused for dead reckoning, with the gyro used to estimate heading drift. During edge cleaning the gyro confirms the robot holds a straight heading even while the wheels slip on purpose to keep pressure against the wall. — [USPTO 9,918,605 "Wall following robot"](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/9918605); [iRobot "Mobile robot for cleaning" US 8,961,695](https://www.freepatentsonline.com/8961695.html)
- **iRobot US 11,662,743 B2 "Robot localization and mapping accommodating non-unique landmarks"** (priority Sep 2020, granted May 2023) is the clearest description of camera-free localization in a commercial cleaner. It uses only bump sensors, short-range IR proximity/obstacle-following sensors, wheel encoders and a 6-axis IMU. Key points:
  - Landmarks are built from wall-following trajectories: straight wall segments, characterized by length, orientation and endpoints, and corners. Poses are recorded at about 1 cm increments while wall following. A segment must be at least 30 cm long to count as a landmark.
  - Because "most household rooms have straight walls... oriented perpendicular to each other," a rectilinear assumption can be applied. A candidate straight segment that matches a known landmark but is offset by less than a threshold angle (example: <10°) triggers a localization correction.
  - Pose drift is tracked as an EKF covariance. When its trace ("uncertainty") exceeds an adaptive threshold U1, the robot runs **active re-localization**: it drives to a landmark chosen for the error component that most needs correcting. A perpendicular approach to a wall fixes the error along the wall normal. A ~45° approach to a suitably oriented landmark can correct Ex, Ey and Eθ together. Re-localization can also be **reactive**, on an opportunistic encounter.
  - Non-unique landmarks (many identical walls) are explicitly allowed.
  — [Google Patents US11662743B2](https://patents.google.com/patent/US11662743B2/en)
- Gyro bias handling in cleaners: a Korean patent (KR100486505B1) stops the robot for a set time, averages gyro output to get a new offset, and corrects the direction error with it. In other words, zero-velocity bias re-estimation. — [Google Patents KR100486505B1](https://patents.google.com/patent/KR100486505B1/en)
- Wall-alignment heading reset in cleaner patents: the robot drives into the wall until the whole front edge (pad) contacts it, then rotates 90° so its axis is parallel to the wall, and only then starts border following. — [search summary of wall-following cleaner patents, e.g. USPTO 8,457,789](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8457789)
- Evolution Robotics Mint (later iRobot Braava) did not rely on onboard range sensing for position. A NorthStar cube projects IR spots on the ceiling. The robot reads their relative orientation to get position and heading, then cleans in systematic back-and-forth lines plus an edging circuit, and builds its map from that position combined with proximity and drop sensors. Without NorthStar it "won't be able to localize itself" and covers a smaller area. One cube per room; more cubes cover more rooms. — [IEEE Spectrum Mint review](http://spectrum.ieee.org/automaton/robotics/home-robots/review-evolution-robotics-mint-sweeper); [Wikipedia: Evolution Robotics](https://en.wikipedia.org/wiki/Evolution_Robotics)
- Neato went the other way and built a cheap laser. Konolige et al., ICRA 2008: a triangulation laser distance sensor with 3 cm accuracy out to 6 m, 10 Hz, 1° resolution over 360°, costing under $30 to build. It became the Neato XV-11 LDS (minimum range about 15 cm). — [ResearchGate: A low-cost laser distance sensor](https://www.researchgate.net/publication/224318681_A_low-cost_laser_distance_sensor); [Hizook](https://www.hizook.com/ultra-low-cost-laser-rangefinders-actualized-neato-robotics/); [RECESSIM wiki](https://wiki.recessim.com/view/Neato_XV-11)
- Measured odometry of a hobby-hacked Roomba (USC EE579 project): after 5 step-forward plus step-back cycles it drifted forward and to the right by hundreds of mm. Rotation angles depended on speed (sometimes over 90°, sometimes under for the same command), and wheel speed could only be set in 20 mm/s steps. The team mapped a 3.8 m × 4.8 m living room on a 200 mm grid with DFS exploration. They judged the map to "closely resemble" the room. RSSI beacon localization (Tmote, Estimote) was "very inaccurate" by comparison. — [USC ANRG Roomba Obstacle Mapping](https://anrg.usc.edu/ee579/spring2016/Roomba/)

### Inferences
- The commercial pattern most transferable to mBot2 is the iRobot non-unique-landmark approach: dead reckoning from gyro plus encoders, a rectilinear-room prior, and occasional deliberate wall/corner encounters that reset specific error components. An HC-SR04 aimed perpendicular at a wall plays the role of their bump/IR adjacency sensor, at range instead of at contact.
- Random-bounce coverage only works because a vacuum's task does not need pose. A mapping robot does, so mBot2 has to correct drift explicitly.
- Commercial products with real maps all added a much better sensor (laser, camera, ceiling beacon) rather than doing clever inference on sonar. That is a hint about the ceiling for single-sonar systems.

### Gaps
- I found no iRobot or Neato paper with measured pose-error numbers for the gyro-plus-landmark (non-camera) system. The patent gives thresholds (30 cm, 10°, 45° approach) but no accuracy figures.
- I could not retrieve Steffen Gutmann's papers on NorthStar accuracy ("Challenges of designing a low-cost indoor localization system using active beacons") in full text, so I have no measured NorthStar accuracy.

## 2. Hobby and educational robots mapping with a single rotating HC-SR04 / LEGO ultrasonic, including particle filters

### Takeaway
Most hobby "single HC-SR04 room mapping" projects are radar-style sweep plotters or pure dead reckoning (encoder plus magnetometer or gyro) with sonar points painted onto a grid. They rarely report accuracy. The few educational particle-filter projects with numbers (LEGO NXT/EV3) used 500 to 1000 particles. They reported 1 to 3 cm in simulation and about 3 to 10 cm in practice-like runs, and needed walls within ±25° of perpendicular and ranges under about 2 m. Specular reflection off oblique walls, which returns maximum range or garbage, is the dominant failure, not range noise.

### Cited Findings
- EV3 localization project (ev3dev; 300 MHz ARM9, 64 MB RAM; 2 ultrasonic sensors, 2 color sensors):
  - 500 particles, initialized at random positions with cardinal orientations. 1000 was tried and judged unnecessarily expensive.
  - The filter ran onboard at **about 10 s per step**.
  - Sensor model: Gaussian with σ = 10 cm (side sensor) and 15 cm (front sensor). Each reading is the minimum of 3 repeated pings. Readings over 200 cm were treated as unreliable, and the sensor needs walls within ±25° of perpendicular.
  - Motion noise: σ = 5 cm position, 0.03 rad heading, 0.05 rad for in-place turns.
  - Results: position errors were typically 3 to 10 cm (one run off by 6.0 cm and 1.4 cm). The particles converged on a global pose after the robot passed distinctive corners. The robot had to start beside a wall and moved by wall following.
  — [jamesjackson/ev3-localization overview](https://github.com/jamesjackson/ev3-localization/blob/master/overview.md)
- LEGO NXT plus ultrasonic MCL (Liu & Fan): 1000 particles (x, y, θ). Gaussian noise is added on each move. Each range is compared with the distance from the particle to the nearest obstacle straight ahead. The paper reports **average accuracy of 1 to 3 cm in simulation** at three indoor map locations (simulation only, not a field result). — [ResearchGate: Mobile Robot Localisation and Navigation Using LEGO NXT and Ultrasonic Sensor](https://www.researchgate.net/publication/328445467_Mobile_Robot_Localisation_and_Navigation_Using_LEGO_NXT_and_Ultrasonic_Sensor); [Semantic Scholar entry](https://www.semanticscholar.org/paper/Mobile-Robot-Localisation-and-Navigation-Using-LEGO-Liu-Fan/27f438af944bbca42981808320909a7e7d23cff8)
- ETH Zürich tests of the NXT ultrasonic sensor:
  - Mean deviation was 0.41 cm against static targets, with a 3 cm minimum range.
  - In dynamic tests, some areas made the sensor read 255 cm (its "no echo" value) instead of the true distance.
  - Between 25 and 50 cm there was "a high probability of returning the wrong value of 48 cm".
  - The field of view is asymmetric because one transducer transmits and the other receives.
  — [ETH SA NXT ultrasonic tests](https://pps.tik.ee.ethz.ch/mindstorms/sa_nxt/tests_us.html)
- HC-SR04 on oblique walls: users report extremely large values (about 2300 cm) when the sensor faces a wall at an angle, which is classic specular loss. — [Arduino Forum](https://forum.arduino.cc/t/hc-sr04-ultrasonic-sensors-incorrect-readings-when-at-an-angle/559813)
- One academic study (only the abstract and snippet were reachable) found the lowest HC-SR04 error at normal incidence for all materials and markedly larger errors at 40° and 150° sensor-to-surface angles. — [ResearchGate: Errors in Distance and Angle Measurements of Ultrasonic Sensor HC-SR04](https://www.researchgate.net/publication/362302904_Errors_in_Distance_and_Angle_Measurements_of_Ultrasonic_Sensor_HC-SR04)
- HC-SR04 range precision against a perpendicular target is good. An Arduino forum test measured an absolute error of about 0.035 cm per cm of range and an SD of 0.1 to 0.5 cm. Other sources report typical ±1 to 2 cm with careful timing versus ±3 to 5 cm naive. The NewPing `ping_median()` discards out-of-range pings and takes the median. — [Arduino Forum accuracy test](https://forum.arduino.cc/index.php?topic=243076.0); [Zbotic](https://zbotic.in/hc-sr04-accuracy-improving-ultrasonic-distance-readings/)
- MapperBot (ESP8266, HC-SR04, QMC5883L magnetometer, 20-step encoder): despite the "SLAM" label, it is dead reckoning from heading plus distance. When it meets an obstacle it rotates 360° and records ranges. No particle filter and no accuracy numbers. — [saques/MapperBot](https://github.com/saques/MapperBot)
- Many GitHub "ultrasonic radar/mapping" projects (HC-SR04 on an SG90 servo, 180° sweep) only plot ranges and do not localize. — [jkang189/Ultrasonic-Servo-Radar](https://github.com/jkang189/Ultrasonic-Servo-Radar); [KarolPWr/Ultrasonic_mapping](https://github.com/KarolPWr/Ultrasonic_mapping); [yichengsun/Room-Mapping-Vehicle](https://github.com/yichengsun/Room-Mapping-Vehicle)
- Classic sonar physics for a rotating sonar:
  - Kuc & Siegel showed that in specular environments sonar returns form circular arcs, which they called "regions of constant depth" (RCDs).
  - Leonard & Durrant-Whyte (1992) used a rotating sonar to extract RCDs and turned them into line (wall) and point (corner/edge) features. Only the beam center normal to a wall, or a corner, returns a consistent range, so walls appear as arcs of near-constant range across the beam width.
  - Kleeman & Kuc concentrated on specular surfaces (smooth walls, bookcases, desks) "that reflect acoustic energy analogous to a mirror."
  — [Leonard thesis/book "Directed Sonar Sensing for Mobile Robot Navigation"](http://web.cecs.pdx.edu/~mperkows/temp/KALMAN/KALMAN-PAPERS/sonar-kalman-ms%20thesis%20-%20kluwer1992.pdf); [Kleeman & Kuc, Mobile Robot Sonar for Target Localization and Classification](https://ecse.monash.edu/centres/irrc/LKPubs/IJRR95.pdf)
- Tardós et al. (IJRR 2002) did robust mapping and localization from sonar data using a ring of 24 sonars and Hough-transform feature extraction. Beevers' summary says they closed loops with an accurate sonar model. — [Tardós et al. PDF](http://diis.unizar.es/biblioteca/00/09/000973.pdf); summarized in [Beevers & Huang ICRA 2006](http://cs.krisbeevers.com/research/slam_icra06.pdf)

### Inferences
- For an mBot2 360° sweep of about 80 readings, expect only the readings near wall normals, plus corners and edges, to be geometrically trustworthy. Oblique-wall readings will often be maximum range or long. The sensor model should therefore treat "no echo / long" as weak or no evidence, not as free space out to that range. Alternatively, fit RCD-style arcs and use the arc center (the wall normal) as the measurement.
- The EV3 experience (σ 10 to 15 cm sensor model, ±25° usable incidence, about 200 cm max, 500 particles) is the closest published analogue to mBot2. It suggests 3 to 10 cm position accuracy is realistic once the filter has converged in a known map. 1 to 3 cm figures come from simulation.
- Taking the minimum of 3 pings (EV3) or a median (NewPing) is common practice to reject spurious long echoes.

### Gaps
- I found no well-documented hobby project with a single HC-SR04 running a particle filter on real hardware that reports measured ground-truth error over room-scale paths. Most hobby projects report no accuracy at all.
- Cross-talk between multiple HC-SR04s is widely discussed, but I found no quantitative source in this pass. It also matters less for a single-sensor mBot2.
- I could not get the full text of the HC-SR04 angle-of-incidence paper (403), so the exact cutoff angle is unconfirmed. The ±25° figure is for the LEGO sensor.

## 3. Research on SLAM with sparse sensing and particle filters on tiny platforms: particle counts and how accuracy scales

### Takeaway
Beevers & Huang showed that five 1-D range readings per scan are enough for particle-filter SLAM if you group many consecutive scans into a "multiscan" using odometry. The cost is many more particles: 100 to 1000 unconstrained, falling to 20 to 40 with a rectilinearity prior. Beevers also ran fixed-point RBPF SLAM with 13 particles on an 8-bit 16 MHz ATmega64 using IR rangefinders. Classic sonar MCL (Fox) needs tens of thousands of samples for global localization and a few hundred or fewer for tracking. Too many samples under real-time limits hurts accuracy, because sensor data gets dropped.

### Cited Findings
- **Beevers & Huang, "SLAM with Sparse Sensing" (ICRA 2006)**:
  - Setup: real Radish laser datasets with all but 5 readings per scan discarded (0°, 45°, 90°, 135°, 180°), maximum range capped at 5 m or 3 m. Scans are grouped into multiscans, and features (lines and segments) are extracted once per multiscan from the odometry-predicted pose history. The filter is a Rao-Blackwellized PF based on FastSLAM 1.0 with Grisetti adaptive sampling.
  - Results:
    - USC SAL (39 × 20 m, 122 m path): 100 particles, 50 scans per multiscan.
    - CMU NSH (25 × 25 m, 114 m path, 3 m range): 600 particles.
    - Stanford Gates (64 × 56 m, 517 m path): 1000 particles, 18 scans per multiscan.
    - Loops closed in all three. Some spurious landmarks came from poor data association.
  - Stated trade-offs:
    - More particles are needed.
    - Success is "sensitive to the amount of pose uncertainty accumulated during a multiscan."
    - Spinning in place is "a difficult situation for sparse sensing because rotation dramatically increases the pose uncertainty and decreases the scan density."
  - The authors' goal was an IR array costing less than US$40.
  — [Beevers & Huang ICRA 2006 PDF](http://cs.krisbeevers.com/research/slam_icra06.pdf)
- **Beevers & Huang, "Inferring and Enforcing Relative Constraints in SLAM" (WAFR 2006)**: adding a rectilinearity prior (walls parallel or perpendicular, tolerance π/10) cut the particles needed from 100 to 20 (USC SAL) and from 600 to 40 (CMU NSH). Runtime on a P4 1.7 GHz fell from 32.0 s to 11.2 s and from 268 s to 34.8 s respectively, averaged over 30 runs. With constraints, the required particle count is "about the same number as needed by many unconstrained slam algorithms that use full laser rangefinder information." — [Beevers & Huang WAFR 2006 PDF](http://cs.krisbeevers.com/research/constraints_wafr06.pdf)
- **Beevers PhD thesis, "Mapping with limited sensing" (RPI 2007), Ch. 7 "Practical SLAM for low-cost robots"**:
  - Platform: the Ratbot, with an ATmega64 (8-bit, 16 MHz, 4 KB SRAM plus 60 KB external SRAM, no FPU), five Sharp GP2D12 IR rangefinders (10 to 80 cm), and 1024 PPR encoders. Encoder noise was modeled as σ = 3% of count.
  - Implementation: 16.16 fixed point, trig lookup tables at 0.45° resolution, hand-written 2×2/3×3 matrix math, and in-place resampling. About 3 KB per particle (up to 125 line landmarks), so **only 13 to 14 particles fit in 44 KB**.
  - Simulated run (15.5 m path, N = 100, m = 180): closed the loop correctly in 91 of 100 trials.
  - Real Ratbot run: a figure-eight in a 2 m × 2.5 m area, 6.25 m path, 35 s, using **only one IR sensor**, N = 13, m = 55. The loop closed.
  - Timing on the ATmega: 0.33 s per motion-model step (about 500 fixed-point multiplies per particle per step) and 0.24 s per SLAM update. That is not real time. The author suggests a simpler (uniform) motion model to get there.
  — [Beevers PhD thesis PDF](http://cs.krisbeevers.com/thesis/krb_phd_thesis.pdf)
- Yap & Shelton (ICRA 2009), "SLAM in large indoor environments with low-cost, noisy, and sparse sonars," combined a multiscan approach with an orthogonality assumption in a particle filter with line segments to map large indoor spaces with cheap sonars. I could not access the full text for numbers. — [ResearchGate abstract](https://www.researchgate.net/publication/221068235_SLAM_in_large_indoor_environments_with_low-cost_noisy_and_sparse_sonars); [Shelton CV listing](https://www.cs.ucr.edu/~cshelton/cv.pdf)
- **Fox, KLD-sampling (IJRR 2003)**:
  - Setup: Pioneer robot, beam-based sonar model, 50 cm × 50 cm × 10° bins. Global localization runs with sonar and with laser both started from 40,000 samples.
  - Sample counts: with sonar the sample count drops more slowly and settles higher in the tracking phase than with laser.
  - Accuracy under real-time limits with sonar: the best average error was 44 cm (KLD) versus 79 cm (likelihood-based adaptive) and 114 cm (best fixed size). These averages include the large errors during initial global localization.
  - Too many samples made "each update... take several seconds", and sensor data had to be discarded.
  - Accuracy versus sample count is U-shaped under real-time constraints.
  — [Fox 2003 PDF (Oxford mirror)](https://www.robots.ox.ac.uk/~cvrg/hilary2005/adaptive.pdf); [SAGE](https://journals.sagepub.com/doi/10.1177/0278364903022012001)
- More particles monotonically lower RMSE in e-puck PF-SLAM studies, but not linearly, and adaptive particle counts can cut particles substantially without loss of accuracy. — [ResearchGate: Particle filter in SLAM using differential drive mobile robot (e-puck)](https://www.researchgate.net/publication/286763834_Particle_filter_in_simultaneous_localization_and_mapping_Slam_using_differential_drive_mobile_robot)
- A modern sparse-sensing follow-up, SoMaSLAM (2024), runs graph SLAM with "soft Manhattan world" constraints on a Crazyflie multi-ranger deck (4 ToF beams) and on 4 or 11 points sampled from 180-point scans. It cites Beevers & Huang as the foundational multiscan RBPF work. — [arXiv 2409.15736](https://arxiv.org/html/2409.15736v1)

### Inferences
- Particle-count guidance for mBot2 in a known 5 cm grid map:
  - Global localization in a single room would plausibly need thousands of particles. KLD-style adaptive sizing (thousands, then a few hundred) is the established pattern.
  - Tracking from a known start needs 100 to 500, based on the EV3 experience and Fox's tracking-phase sizes.
  - With a Manhattan/rectilinear heading prior, Beevers' 5× to 15× reduction suggests tens of particles may suffice for heading-constrained tracking.
- An mBot2 360° sweep of about 80 readings already is a dense "multiscan" taken from (nearly) one pose. That is better than Beevers' moving multiscans, because the odometry error inside the sweep is only rotational and the gyro bounds it. Beevers' warning still applies: rotation degrades sparse sensing, so integer-degree gyro quantization and turn-rate effects inside a sweep matter.
- Fox's 44 cm averages are for global localization from scratch in large office spaces and should not be read as the expected room-scale tracking error.

### Gaps
- I found no paper with explicit particle-count versus accuracy curves for a single rotating sonar at room scale. The numbers above are from multi-sonar rings, 5-beam IR, or LEGO sensors.
- I could not retrieve numbers for Khepera or Pololu particle-filter localization in this pass.

## 4. Lightweight techniques: landmark relocalization, docking and beacons, bump constraints, active sensing, wall-touch heading resets

### Takeaway
In practice, cheap robots bound drift with three things: (a) a rectilinear-room heading prior plus gyro bias re-estimation while stationary, (b) deliberate wall and corner encounters that correct specific pose components (perpendicular approach for distance, a straight wall run for heading), and (c) external beacons or a dock when absolute position is needed. Active re-localization (go to the landmark that reduces the largest error component) is patented and used commercially by iRobot.

### Cited Findings
- Active and reactive re-localization against straight-wall and corner landmarks:
  - Triggered when the EKF uncertainty trace exceeds an adaptive threshold.
  - The landmark is chosen by which error component (Ex, Ey, Eθ) needs correcting. A 45° approach can correct all three, and a perpendicular approach corrects the distance-normal component.
  - Landmarks must be at least 30 cm long, and the heading-match tolerance for applying a correction is under 10°.
  — [iRobot US11662743B2](https://patents.google.com/patent/US11662743B2/en)
- Bump and adjacency events serve as position constraints. The same patent treats bump sensors, obstacle-following IR, break-beam and capacitive sensors as "adjacency sensors" whose contacts define landmark trajectories. — [iRobot US11662743B2](https://patents.google.com/patent/US11662743B2/en)
- "Touch the wall" alignment: drive into the wall until the full front edge contacts it (mechanically squaring the heading), then rotate 90° to run parallel before border following. — [Wall-following robot cleaner patents, e.g. US 8,457,789](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8457789)
- Gyro bias reset: stop for a set time, average the gyro output to get a new offset, then correct accumulated direction error. — [KR100486505B1](https://patents.google.com/patent/KR100486505B1/en)
- Rectilinear and Manhattan priors cut particle needs by 5× to 15× in sparse-sensing SLAM (see §3). — [Beevers & Huang WAFR 2006](http://cs.krisbeevers.com/research/constraints_wafr06.pdf)
- External beacon approach: NorthStar IR ceiling spots give absolute position and heading per room, with one cube per room. — [IEEE Spectrum Mint review](http://spectrum.ieee.org/automaton/robotics/home-robots/review-evolution-robotics-mint-sweeper); [Evolution Robotics NorthStar patent family, e.g. US 8,452,450](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8452450)
- RSSI beacons (Tmote, Estimote BLE) were far worse than odometry-based grid mapping in a 3.8 × 4.8 m room. — [USC ANRG Roomba project](https://anrg.usc.edu/ee579/spring2016/Roomba/)
- The EV3 MCL converged on global pose only after the robot passed distinctive corners, and it required starting along a wall. — [ev3-localization overview](https://github.com/jamesjackson/ev3-localization/blob/master/overview.md)
- Sonar geometry favors this kind of active sensing. Walls return reliably only near normal incidence (RCDs), and corners and edges are the other reliable returns. — [Leonard & Durrant-Whyte, Directed Sonar Sensing](http://web.cecs.pdx.edu/~mperkows/temp/KALMAN/KALMAN-PAPERS/sonar-kalman-ms%20thesis%20-%20kluwer1992.pdf)

### Inferences
- mBot2 analogues of the commercial tricks:
  - **Heading:** during a sweep, find the minimum-range direction toward a known wall. That is the wall normal. With the HC-SR04 cone this is usable to a few degrees by fitting the arc of the range-versus-angle curve, not by taking the single minimum. Snap the heading to the Manhattan axes when the difference is under about 10° (iRobot's threshold).
  - **Distance:** turn to face a wall squarely and take a median of pings. This gives cm-level range along the normal, the same as iRobot's perpendicular-approach correction.
  - **Corners:** two perpendicular wall normals in one sweep fix x, y and θ at once.
  - **Gyro bias:** re-estimate it while stationary before each sweep.
  - **Bumps:** treat a bump as a hard constraint that the robot front is at a known occupied cell boundary.
- A "home" pose, such as a start corner or a tape mark that the user places the robot on, works as a cheap dock-equivalent absolute reset.

### Gaps
- I found no quantitative published data on heading accuracy from sonar wall-normal fitting with an HC-SR04-class cone. It would need to be measured on the mBot2.
- I did not find accuracy numbers for Roomba Home Base / IR dock homing.

## 5. Compute budgets: particle filters and scan matching in JavaScript/browser and on microcontrollers

### Takeaway
Compute is not the bottleneck for mBot2. One sweep takes about 9 s, so even a few hundred ms per filter update is negligible. On an Apple M4 Max under Node 22 (V8, the same engine as Chrome), my local micro-benchmark scored 80 beams per particle at:
- about 1.6 µs per particle with a precomputed likelihood field (1000 particles in 1.6 ms),
- about 9 µs per particle with single-ray casting,
- about 61 µs per particle with a 7-ray cone model.

Microcontroller budgets are the opposite extreme: Beevers fit only 13 particles on an ATmega64 at about 0.3 s per step. The CyberPi's ESP32 would sit between these, but running the filter there is unnecessary when the browser does the work.

### Cited Findings
- Local benchmark (run for this research; `bench.mjs` in the session scratchpad). It is one measurement on one machine, not a published source. Setup: 8 m × 8 m map at 5 cm (160 × 160 cells), 80 beams per update, maximum range 150 cm. Results:

  | Model | 200 particles | 500 | 1000 | 2000 | 5000 |
  |---|---|---|---|---|---|
  | Likelihood field (distance-transform lookup) | 0.32 ms | 0.79 ms | 1.6 ms | 3.2 ms | 8.2 ms |
  | Single-ray cast, 2.5 cm step | 1.8 ms | 4.5 ms | 9.1 ms | 18 ms | 46 ms |
  | 7-ray cone across about 26°, minimum range | 12.7 ms | 31 ms | 61 ms | 124 ms | 305 ms |

  Expect roughly 2× to 5× slower on a mid-range laptop or phone browser (rough estimate, not measured).
- CDDT (Walsh & Karaman, 2017), a compressed directional distance transform for 2D ray casting on occupancy grids, maintains **2500 particles × 61 ray casts each at 40 Hz on one CPU thread** onboard a mobile robot (C++/RangeLibc). — [arXiv 1705.01167](https://arxiv.org/abs/1705.01167)
- In-browser particle filter demos exist (for example a 500-particle, 4-landmark demo where each update is 2000 likelihood computations), but they publish no timing. — [jameseoconnor/particle-filter-demo](https://github.com/jameseoconnor/particle-filter-demo); [tommycohn.com Demo Particle Filter](http://tommycohn.com/Demo-Particle-Filter/index.html); [GitHub topic particle-filter (JavaScript)](https://github.com/topics/particle-filter?l=javascript)
- Generic JavaScript throughput: hobby benchmarks simulate millions of simple particles per frame in plain JS with typed arrays, which shows V8 does tens to hundreds of millions of simple float ops per frame. — [dgerrells blog: Simulating 20,000,000 particles](https://dgerrells.com/blog/how-fast-is-javascript-simulating-20-000-000-particles); [HN discussion](https://news.ycombinator.com/item?id=40902012)
- Microcontroller (8-bit) reference point: ATmega64 at 16 MHz with 16.16 fixed point gave 13 particles, 0.33 s per prediction step and 0.24 s per update. Memory, not CPU, capped the particle count (about 3 KB per particle for FastSLAM landmarks). — [Beevers thesis Ch. 7](http://cs.krisbeevers.com/thesis/krb_phd_thesis.pdf)
- ARM9 Linux (EV3, 300 MHz): 500 particles took about 10 s per step in their (likely Python) implementation. — [ev3-localization](https://github.com/jamesjackson/ev3-localization/blob/master/overview.md)
- Older Pentium-class PCs with sonar MCL: updates with very large sample sets took "several seconds". This is why adaptive sample sizes mattered. — [Fox 2003](https://www.robots.ox.ac.uk/~cvrg/hilary2005/adaptive.pdf)

### Inferences
- For mBot2 in the browser, one update per 9 s sweep with 80 beams could afford:
  - 5,000 to 20,000 particles with a likelihood field in under 50 ms, or
  - 1,000 to 2,000 particles with a proper cone model in about 100 ms.

  That leaves plenty of headroom for global localization in a room, KLD-style adaptive sizing, or several hypotheses. Running it in a Web Worker keeps the UI responsive. WebAssembly is unnecessary at these sizes.
- The likelihood-field model is about 40× cheaper than a cone ray-cast. It handles specular "max-range" returns more gracefully if max-range readings are simply skipped. The cone model is more faithful to the about 26° HC-SR04 beam, because the nearest surface anywhere in the cone produces the echo. A practical compromise is to precompute, per grid cell and per heading bin, the expected cone minimum range: 160 × 160 × 72 bins is about 1.8 M entries, about 3.7 MB as Uint16. That makes the cone model as cheap as a lookup.
- Scan-to-map matching (grid search over x, y, θ around the odometry pose, for example ±15 cm in 2.5 cm steps × ±10° in 1° steps, about 3,500 poses × 80 beams) also costs a few ms to tens of ms in JS, based on the per-beam costs above. It is a viable deterministic alternative to a particle filter for tracking.

### Gaps
- I did not find any published benchmark of particle filter or scan matching timing specifically in a browser or WebAssembly for robot localization. The JS numbers here come from one local Node/V8 run on a fast desktop CPU and should be re-measured on the target browser and device.
- I did not find published ESP32 particle-filter timing figures in this pass.
