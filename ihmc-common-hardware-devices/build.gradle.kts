plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   group = "us.ihmc"
   version = "0.0.2"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-common-hardware.git"
   openSource = false

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-sensor-processing:0.14.0-250815")
   api("us.ihmc:ihmc-robotics-toolkit:0.14.0-250815")
   api("us.ihmc:ihmc-hardware-status-ui:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
