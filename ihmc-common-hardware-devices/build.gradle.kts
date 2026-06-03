plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-sensor-processing:source")
   api("us.ihmc:ihmc-hardware-status-ui:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
