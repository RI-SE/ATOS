 pipeline {
    agent any

    environment {
        GIT_SSH_COMMAND = 'ssh -i /home/u93029@sp.se/github/key'
    }

    stages {
        stage('Build') {
            steps {
                sh 'printenv'
                sh 'git submodule update --init --recursive'
            }
        }
    }
} 
