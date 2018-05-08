#!/bin/bash

echo `git add .`
echo `git commit -m "${*}"`
echo `git push origin master`