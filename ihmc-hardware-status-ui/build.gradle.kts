plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   group = "us.ihmc"
   version = "0.0.1"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-common-hardware.git"
   openSource = false

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-ethercat-master:0.16.0")
   api("us.ihmc:scs2-simulation-construction-set:17-0.30.0")
   api("us.ihmc:ihmc-java-toolkit:0.14.0-241016")
   api("us.ihmc:ihmc-hardware-xml-toolkit:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
