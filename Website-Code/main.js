function startCountdown(targetDateStr, elemIdPrefix) {
  const target = new Date(targetDateStr).getTime();
  function update() {
    const now = Date.now();
    const diff = target - now;
    if (diff <= 0) {
      document.getElementById(elemIdPrefix + '-days').textContent = '00';
      document.getElementById(elemIdPrefix + '-hours').textContent = '00';
      document.getElementById(elemIdPrefix + '-minutes').textContent = '00';
      document.getElementById(elemIdPrefix + '-seconds').textContent = '00';
      return;
    }
    const days = Math.floor(diff / (1000*60*60*24));
    const hours = Math.floor((diff % (1000*60*60*24)) / (1000*60*60));
    const minutes = Math.floor((diff % (1000*60*60)) / (1000*60));
    const seconds = Math.floor((diff % (1000*60)) / 1000);
    document.getElementById(elemIdPrefix + '-days').textContent = String(days).padStart(2,'0');
    document.getElementById(elemIdPrefix + '-hours').textContent = String(hours).padStart(2,'0');
    document.getElementById(elemIdPrefix + '-minutes').textContent = String(minutes).padStart(2,'0');
    document.getElementById(elemIdPrefix + '-seconds').textContent = String(seconds).padStart(2,'0');
  }
  update();
  setInterval(update, 1000);
}

document.addEventListener('DOMContentLoaded', () => {
const menuToggle = document.getElementById('menuToggle');
const sidebar = document.querySelector('.sidebar');
const sidebarClose = document.getElementById('sidebarClose');

// Opens/closes from the main header button
menuToggle.addEventListener('click', () => {
  sidebar.classList.toggle('active');
});

// Closes when clicking the new "X" inside the sidebar
sidebarClose.addEventListener('click', () => {
  sidebar.classList.remove('active');
});
});

let currentSlide = 0;

function moveSlide(direction) {
  const slides = document.querySelectorAll('.carousel-slide img');
  
  // Remove the active class from the current image
  slides[currentSlide].classList.remove('active');
  
  // Calculate the next image index
  currentSlide += direction;
  
  // Loop back around if you reach the end or the beginning
  if (currentSlide >= slides.length) {
    currentSlide = 0;
  } else if (currentSlide < 0) {
    currentSlide = slides.length - 1;
  }
  
  // Add the active class to the new image
  slides[currentSlide].classList.add('active');
}