
pipeline {
	agent any

	options {
		timeout(time: 15, unit: 'MINUTES')
	}
	stages {
		stage('Build') {
			steps {
				sh 'echo "Executing build steps..."'
				cmake installation: 'InSearchPath'
				cmakeBuild
					buildDir: 'build',
					installation: 'InSearchPath',
					steps: [
						[args: 'all', envVars: 'DESTDIR=${WORKSPACE}/artifacts'] 
					]
			}
		}
		stage('Run tests') {
			parallel {
				stage('Integration tests') {
					steps {
						sh 'echo "Running Maestro integration tests..."'
						sh './allMaestroIntegrationTests.sh'
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


