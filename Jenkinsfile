
pipeline {
    agent { docker { image 'python:3.5.1' } }
    stages {
        stage('build') {
            steps {
                sh 'echo "Executing build script..."'
                sh './buildMaestro.sh'
            }
        }
    }
}


