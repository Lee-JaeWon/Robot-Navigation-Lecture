#!/bin/bash

RIDERS_CHALLENGE_ID=67
RIDERS_API_HOST=https://api.riders.ai
RIDERS_F1TENTH_HOST=https://f1tenth.riders.ai

printf "\nRetrieving your API Token...\n"

RED='\033[0;31m'
NC='\033[0m' # No Color

if [[ -z ${RIDERS_AUTH_TOKEN} ]]; then
  DATA='{"username": '\""$RIDERS_USERNAME"\"', "password": '\""$RIDERS_PASSWORD"\"'}'

  COLAB_HOST="$RIDERS_API_HOST"'/get-token/'
  COLAB_TOKEN_RESPONSE=$(curl -H "Content-Type: application/json" -X POST -d "$DATA" "$COLAB_HOST")
  STATUS_CODE=$(curl --write-out '%{http_code}' --silent --output /dev/null -H "Content-Type: application/json" -X POST -d "$DATA" "$COLAB_HOST")

  if [ $STATUS_CODE == "404" ]; then
    echo -e "${RED}Authorization Failed: Token Not Found"
    exit 1
  elif [ $STATUS_CODE == "403" ]; then
    echo "${RED}Too Many Attemps, Please contact with Riders.ai admins"
    exit 1
  elif [ $STATUS_CODE == "200" ]; then
    echo "Authorization Successful"
  fi

  RIDERS_AUTH_TOKEN=$(echo $COLAB_TOKEN_RESPONSE | jq -c '.token' | tr -d '"')
  if [[ -z ${RIDERS_AUTH_TOKEN} ]]; then
    printf "Faulty Auth Token, please try restarting this script.\n"
    exit
  fi
fi

# We assume this script was placed on the project root. Otherwise we should change directory.
# Archive tar file with arbitrary filename
printf "\n\nBuilding TAR GZ file, this may take a while...\n"
cd /project
echo ${RIDERS_AUTH_TOKEN} > ${TOKEN_FILE}

git ls-files | tar Tzcf - challengeproject.tar.gz
printf "Compiled TAR GZ file for submission.\n"

RIDERS_API_SUBMIT="${RIDERS_API_HOST}/api/v1/challenge/${RIDERS_CHALLENGE_ID}/submit/"

curl --location \
    -F 'file=@./challengeproject.tar.gz' \
    -F description="$DESCRIPTION" \
    -H "Authorization: TOKEN ${RIDERS_AUTH_TOKEN}" \
    "${RIDERS_API_SUBMIT}"