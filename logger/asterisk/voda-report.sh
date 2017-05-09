#!/bin/sh
# Anything - first word of speech will be lost
echo "o."
mysql -u voda --password=vodenjak voda <<EOF
/* Counter status */
SELECT volume as 'Status:' FROM voda ORDER BY datetime DESC LIMIT 1;
/* Today */
SELECT
  ROUND(t1.value - t2.value, 2) AS 'Today:'
FROM
  (SELECT DATE(datetime) AS dt, MAX(volume) AS value FROM voda GROUP BY dt) t1
JOIN
  (SELECT DATE(datetime) AS dt, MAX(volume) AS value FROM voda GROUP BY dt) t2
    ON t1.dt = t2.dt + INTERVAL 1 DAY
ORDER BY t1.dt DESC
LIMIT 1;
/* Yesterday */
SELECT
  ROUND(t1.value - t2.value, 2) AS 'Yesterday:'
FROM
  (SELECT DATE(datetime) AS dt, MIN(volume) AS value FROM voda GROUP BY dt) t1
JOIN
  (SELECT DATE(datetime) AS dt, MIN(volume) AS value FROM voda GROUP BY dt) t2
    ON t1.dt = t2.dt + INTERVAL 1 DAY
ORDER BY t1.dt DESC
LIMIT 1;
EOF
