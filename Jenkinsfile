
pipeline {
	agent any

	options {
		timeout(time: 15, unit: 'MINUTES')
	}
	stages {
		stage('Build') {
			steps {
				sh 'echo "Executing build steps..."'
				cmakeBuild cleanBuild: true, buildDir: 'build', installation: 'InSearchPath', steps: [[envVars: 'DESTDIR=${WORKSPACE}/artifacts', withCmake: true]]
			}
		}
		stage('Run tests') {
			parallel {
				stage('Master Integration tests') {
					when{
						expression {
							GIT_BRANCH = 'origin/' + sh(returnStdout: true, script: 'git rev-parse --abbrev-ref HEAD').trim()
							return GIT_BRANCH == 'origin/master'
						}	
					}
					steps {
						sh 'echo "Running extensive Maestro integration tests..."'
						sh './maestroExtensiveTests.sh'
					}
				}
				stage('Dev Integration tests') {
					steps {
						sh 'echo "Running standard Maestro integration tests..."'
						sh './maestroStandardTests.sh'
					}
				}
				stage('Format check') {
					steps {
						sh 'echo "Running code formatting check..."'
						sh './checkCodeFormat.sh'
					}
				}
	    		}
		}
	}
}

