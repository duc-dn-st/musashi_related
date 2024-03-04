pm2 start start_standby_mode.sh

pm2 start app.sh --no-autorestart --restart-delay 30000

pm2 save