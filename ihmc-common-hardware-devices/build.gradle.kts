plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
    loadProductProperties("../product.properties")

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
