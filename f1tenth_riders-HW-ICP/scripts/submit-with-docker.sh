#!/bin/bash

set -o errexit

printf "\n\nBuilding your Agent Docker image, this may take a while...\n"

# Try to build agent locally to prevent sending user_projects that fail on build process.
docker-compose -f docker-compose.yml build agent

printf "Image built & tagged successfully.\n"

unset RIDERS_USERNAME
unset RIDERS_PASSWORD
unset DESCRIPTION
unset RIDERS_AUTH_TOKEN
unset TOKEN_FILE

SCRIPT_DIR=$(dirname "$0")
TOKEN_FILE="${SCRIPT_DIR}/../.riders-token"
export TOKEN_FILE

printf "\n\nThis script will compile your project & upload to Riders.ai for evaluation.\n"

if [ -e ${TOKEN_FILE} ]
then
  RIDERS_AUTH_TOKEN=$(cat ${TOKEN_FILE})
  export RIDERS_AUTH_TOKEN
  printf "Found token, continuing...\n"
else
  printf "Please enter your Riders.ai Username & Password\n\n"

  echo -n "Username:"
  read RIDERS_USERNAME
  echo -n "Password:"
  read -s RIDERS_PASSWORD
  
  export RIDERS_USERNAME
  export RIDERS_PASSWORD
fi

printf "\n\nPlease enter a short description of your submission. This would help you compare separate submissions\n"
echo -n "Description:"
read DESCRIPTION

export DESCRIPTION

docker-compose -f scripts/submit-docker-compose.yml build submit
docker-compose -f scripts/submit-docker-compose.yml up submit

printf "Submission successful! Please go to https://riders.ai/challenge/67/submissions to validate."