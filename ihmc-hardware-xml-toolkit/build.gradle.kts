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
   api("xml-apis:xml-apis:2.0.2")
   api("com.sun.xml.bind:jaxb-impl:4.0.5")
   api("us.ihmc:euclid-geometry:0.22.5")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
