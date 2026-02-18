plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   loadProductProperties("../group.product.properties")

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-ethercat-master:0.16.1")
   api("us.ihmc:scs2-simulation-construction-set:17-0.32.0")
   api("us.ihmc:ihmc-java-toolkit:0.14.0-241016")
   api("us.ihmc:ihmc-robotics-toolkit:20251002")
   api("us.ihmc:ihmc-hardware-xml-toolkit:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
