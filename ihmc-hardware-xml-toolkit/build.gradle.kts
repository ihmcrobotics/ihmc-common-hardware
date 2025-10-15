plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
    loadProductProperties("../product.properties")

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
