pipeline {
  agent any
  stages {
    stage('Build') {
			steps {
				sh 'echo "Executing build steps..."'
				cmakeBuild cleanBuild: true, buildDir: 'build', installation: 'InSearchPath', steps: [[envVars: 'DESTDIR=${WORKSPACE}/artifacts', withCmake: true]]
			}
    }
  }
}
