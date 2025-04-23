#!/bin/bash

# 원본 파일 이름
input="test.mp3"

# 트랙 리스트
declare -a tracks=(
    "00:00 2:47 Johnny Sky - Sudor y Calor.mp3"
    "02:47 2:28 Carlos Baute - Muy Loco.mp3"
    "05:15 2:29 KHEA - COMO YO TE QUIERO.mp3"
    "07:44 2:33 CHARLY & JOHAYRON - Dos Amigos.mp3"
    "10:17 3:45 Ralphy Dreamz X Mayinbito - A Mil Pedazos.mp3"
    "14:02 2:41 Ochoa - Dime Que Somos.mp3"
    "16:43 3:10 Otro Segundo - Vinny Rivera x DJ Nico.mp3"
    "19:53 4:01 Lean&Akai - Para Besarte.mp3"
    "23:54 2:54 Pinto Picasso - Feeling Something.mp3"
    "26:48 3:05 Break Out The Crazy - Notas.mp3"
    "29:53 3:43 Johandy - Que Duela.mp3"
    "33:36 3:59 DJ Clau & Pablo Dazan - Locura.mp3"
    "37:35 2:46 DJ Dorin Bachata - Estar Conmigo.mp3"
    "40:21 3:34 Róman & DJ Khalid - Luna.mp3"
    "43:55 3:31 Marco Puma & Pinto Picasso - Unico Grande Amore.mp3"
    "47:26 2:57 Sebas Garreta x Dave Aguilar x Dj Husky - Frío en el Infierno.mp3"
    "50:23 3:21 Dani J - Piscina.mp3"
    "53:44 3:16 DJ Nassos B & Pablo Dazan - dEsANiMaO.mp3"
    "57:00 3:36 59Pinto Picasso & Jensen - Un Trago Una Bachata.mp3"
    "01:00:36 3:50 Chalres Luis & DJ Nico - El Tonto.mp3"
)

# 트랙 분할
for track in "${tracks[@]}"; do
    IFS=' ' read -r start duration name <<< "$track"
    ffmpeg -i "$input" -ss "$start" -t "$duration" -c copy "$name"
done
