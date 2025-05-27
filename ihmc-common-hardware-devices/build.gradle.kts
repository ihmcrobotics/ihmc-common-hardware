plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   group = "us.ihmc"
   version = "0.0.1"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-common-hardware-modules.git"
   openSource = false

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-robotics-tools-joint-kinematics:0.15.3")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-realtime:1.7.0")
   api("us.ihmc:ihmc-robot-models:0.21.9")
   api("us.ihmc:open-alexander:source")
   api("us.ihmc:ihmc-ethercat-master:0.16.0")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
