
 
<a name="P10-MAP"></a>

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->

[![Contributors][contributors-shield]][contributors-url]
[![Stars][stars-shield]][stars-url]
[![CC BY 4.0][cc-by-shield]][cc-by]


<!-- PROJECT LOGO -->
<br />
<div align="center">
  <!-- <a href="https://github.com/AndreiVoica/P10-MAP/">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a> -->

  <h3 align="center">P10 - Material Acceleration Platforms</h3>

  <p align="center">
    Master Thesis in Robotics - Aalborg University
    <br />
    <a href="https://www.youtube.com/playlist?list=PLTbrI-WjdIEfSyzKvvQM6LQMU2solaiKI">View Demo</a>
    <br />
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About the Project

<p align="center">
<img src="/docs/imgs/Frontpage.png" alt="Frontpage" width="700">
</p>


This project focuses on the transformation of chemistry laboratories into autonomous environments that can accelerate the discovery of new materials. The main goal is to optimize chemical processes that are typically performed by humans and can thus be slow and prone to errors.

The project utilizes robotic solutions and simulation to achieve this goal. The autonomous laboratory will be implemented on the AAU Matrix Production setup. This setup consists of five Kuka robotic manipulators, the B&R Automation Acopos 6D magnetic levitation platform, and various custom-made parts.

For development purposes, Nvidia Isaac Sim is used to create a simulated environment that replicates the physical setup. This allows for the execution of different experiments in a virtual setting. The Robot Operating System (ROS1) is used to control both the simulated Kuka manipulators and their real-world counterparts.

The simulation experiments demonstrate that the system is capable of automatically completing a chemical process. However, transferring these capabilities to the physical setup poses a significant challenge.

The project is the outcome of a Master's thesis in Robotics at Aalborg University. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

Ubuntu 20.04 together with Isaac Sim 2022.2.1 and ROS Noetic was used for this project.

<p align="left"> 
 
<a href="https://www.python.org" target="_blank"> <img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg" alt="python" width="40" height="40"/> </a> 
<a href="https://www.w3schools.com/cpp/" target="_blank"> <img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/cplusplus/cplusplus-original.svg" alt="cplusplus" width="40" height="40"/> </a>
<a href="https://www.overleaf.com/"> <img src="https://images.ctfassets.net/nrgyaltdicpt/h9dpHuVys19B1sOAWvbP6/5f8d4c6d051f63e4ba450befd56f9189/ologo_square_colour_light_bg.svg" alt="overleaf_logo" width="40" height="40"> </a>
<a href="https://git-scm.com/" target="_blank"> <img src="https://www.vectorlogo.zone/logos/git-scm/git-scm-icon.svg" alt="git" width="40" height="40"/> </a> 
<a href="https://www.linux.org/" target="_blank"> <img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/linux/linux-original.svg" alt="linux" width="40" height="40"/> </a> 
<a href="https://www.nvidia.com/en-us/omniverse/" target="_blank"> <img src="https://docs.omniverse.nvidia.com/con_connect/_images/renderer.png" alt="OmniverseIsaacSim" width="40" height="40"/> </a>
<a href="https://www.ros.org/"> <img src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg" alt="ros_logo" height="36"> </a>

</p>
 
 
<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started


To get a local copy up and running follow these example steps.

### Prerequisites
[06/06/2023]

* Isaac Sim requirements:
Some minimum requirements are needed to install Isaac Sim, check the [Link](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/requirements.html) for more details.

| Element | Minimum Spec                       | Good            | Ideal                                               |
|---------|------------------------------------|----------------|-----------------------------------------------------|
| OS      | Ubuntu 20.04/22.04, Windows 10/11  | Ubuntu 20.04/22.04, Windows 10/11 | Ubuntu 20.04/22.04, Windows 10/11 |
| CPU     | Intel Core i7 (7th Generation), AMD Ryzen 5 | Intel Core i7 (9th Generation), AMD Ryzen 7 | Intel Core i9, X-series or higher, AMD Ryzen 9, Threadripper or higher |
| Cores   | 4                                  | 8              | 16                                                  |
| RAM     | 32GB*                              | 64GB*          | 64GB*                                               |
| Storage | 50GB SSD                           | 500GB SSD      | 1TB NVMe SSD                                        |
| GPU     | GeForce RTX 2070                   | GeForce RTX 3080| RTX A6000                                           |
| VRAM    | 8GB*                               | 10GB*          | 48GB*                                               |


Note: GeForce RTX 2060 6GB VRAM is also compatible.

Note: The asterisk (*) indicates that the specified amount is the minimum required, but more is recommended for better performance.


### Installation

* Isaac Sim and MAPs Extension
* ROS
* MoveIt
* KukaVarProxy 
* Planar Motor Controller API

1. To install Isaac Sim, follow the instructions in the [Isaac Sim documentation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html#).
Once Isaac Sim is installed follow the steps in [MAPs Extension](/docs/installation/MAPs_Extension/README.md)

2. To install ROS, follow the instructions in the [ROS Noetic documentation](http://wiki.ros.org/noetic/Installation/Ubuntu)

3. To install MoveIt, follow the instructions in the [MoveIt documentation](https://moveit.ros.org/install/)

4. To install the KukaVarProxy, follow the instructions in the [KukaVarProxy documentation](https://github.com/ImtsSrl/KUKAVARPROXY)

5. To install the Planar Motor Controller PMC API, follow the instructions in the [planar motor controller API documentation](/docs/installation/planar_motor_control_API/README.md)


<!-- USAGE EXAMPLES -->
## Usage

The following image shows the communication workflow between ROS and physical robots (blue), Simulation environment
(green) and Magnetic levitation platform (orange). Machine Readable Recipe is not implemented.

<p align="center">
<img src="/docs/imgs/Workflow.drawio_v2.png" alt="Workflow Diagram" width="400">
</p>



<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING 
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

-->


<!-- LICENSE -->
## License

This work is licensed under a
[Creative Commons Attribution 4.0 International License][cc-by].

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Daniel Moreno - [LinkedIn](https://www.linkedin.com/in/daniel-mparis/) - danimp94@gmail.com
Andrei Voica - [LinkedIn](https://www.linkedin.com/in/andrei-voica-825b7a104/) - avoica18@student.aau.dk

Project Link: [https://github.com/AndreiVoica/P10-MAP](https://github.com/AndreiVoica/P10-MAP)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

Use this space to list resources you find helpful and would like to give credit to. I've included a few of my favorites to kick things off!

* [Choose an Open Source License](https://choosealicense.com)
* [GitHub Emoji Cheat Sheet](https://www.webpagefx.com/tools/emoji-cheat-sheet)
* [Malven's Flexbox Cheatsheet](https://flexbox.malven.co/)
* [Malven's Grid Cheatsheet](https://grid.malven.co/)
* [Img Shields](https://shields.io)
* [GitHub Pages](https://pages.github.com)
* [Font Awesome](https://fontawesome.com)
* [React Icons](https://react-icons.github.io/react-icons/search)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/AndreiVoica/P10-MAP.svg?style=for-the-badge
[contributors-url]: https://github.com/AndreiVoica/P10-MAP/graphs/contributors
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/AndreiVoica/P10-MAP/stargazers
[issues-shield]: https://img.shields.io/github/issues/AndreiVoica/P10-MAP.svg?style=for-the-badge

[issues-url]: https://github.com/othneildrew/Best-README-Template/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Python-url]: https://nextjs.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 


[![CC BY 4.0][cc-by-image]][cc-by]

[cc-by]: http://creativecommons.org/licenses/by/4.0/
[cc-by-image]: https://i.creativecommons.org/l/by/4.0/88x31.png
[cc-by-shield]: https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg?style=for-the-badge