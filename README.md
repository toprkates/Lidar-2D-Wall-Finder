## Lidar Data Reader and Processor
> This project is done for Kocaeli University Programming Lab by Toprak Ateş and Melih Eren Mallı.
The Project reads pre-structured raw lidar datas and processes them by various ways. Given dots are represented by the custom graph, and placed by their given polar coordinates. The dots, then, are used by our RANSAC algorithm to find the best lines that dots create. These lines are seen as walls that robot cannot pass. These dynamic lines, then, are checked if they intersect with each other using various linear algebric processes. If they do, the intersection points are placed with a distinct mark; just as our professor wanted. Every function is defined with long comments. We tried our best to make the program as modular and dynamically usable as possible. 

## Requirements
SFML Library for C++
Libcurl Library for C++

if you have any questions about the project, please contact us via our mails. 
