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
   api("us.ihmc:scs2-simulation-construction-set:17-0.33.0")
   api("us.ihmc:ihmc-hardware-xml-toolkit:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
