plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-sensor-processing:20251002") {
      exclude(group = "us.ihmc", module = "ros2-library")
      exclude(group = "us.ihmc", module = "ros2-common-interfaces")
      exclude(group = "us.ihmc", module = "ihmc-interfaces")
      exclude(group = "us.ihmc", module = "ihmc-pub-sub")
      exclude(group = "us.ihmc", module = "ihmc-pub-sub-serializers-extra")
      exclude(group = "us.ihmc", module = "ihmc-pub-sub-xjc")
   }
   api("us.ihmc:ihmc-hardware-status-ui:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
