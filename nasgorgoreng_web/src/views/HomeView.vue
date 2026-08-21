<script setup>
import Members from '@/components/home/Members.vue'
</script>

<template>
  <div class="container members-container col-xxl-8 px-4 py-5 vh-100">
    <div class="row flex-lg-row-reverse align-items-center g-5 pt-5">
      <div
        class="col-10 col-sm-8 col-lg-6"
        data-aos="fade-up"
        data-aos-delay="500"
        data-aos-duration="1000"
      >
        <img
          src="/logo-dark.png"
          class="d-block mx-lg-auto img-fluid float-animation"
          alt="NasgorGoreng"
          width="700"
          height="500"
          loading="lazy"
        />
      </div>
      <div class="col-lg-6" data-aos="fade-up" data-aos-delay="250" data-aos-duration="500">
        <h1 class="display-5 fw-bold lh-1 mb-3">
          From
          <span class="text-danger">Nasi Goreng</span>
          to <span class="text-info">IRIS</span>.
        </h1>
        <p class="lead">
          Fighting for the first place in IRIS CUP 2025,
          <strong>NasgorGoreng</strong> design an optimal autonomous
          <strong>line follower robot</strong>, equipped with panel control and telemetry system. We
          do our best to be the next IRIS 2025.
        </p>
        <div class="d-grid gap-2 d-md-flex justify-content-md-start">
          <a type="button" href="#members" class="btn btn-light btn-lg px-4"> Meet Our Team </a>
          <router-link to="/station" class="btn btn-outline-info btn-lg px-4">
            Base Station
          </router-link>
        </div>
      </div>
    </div>
  </div>
  <div id="members" class="bg-white text-black">
    <div class="container">
      <div class="col text-center py-5" data-aos="fade-up" data-aos-duration="1000">
        <h1><i>Behind The Wheel.</i></h1>
        <p class="lead">
          Meet the dedicated team members who drive NasgorGoreng's success in IRIS CUP 2025.
        </p>
      </div>
    </div>

    <div class="container">
      <div class="row">
        <div class="col-md-4">
          <Members
            name="Hendra Manudinata"
            major="Information Technology"
            role="Programmer"
            instagram="manoedinata"
            image="/members/2.png"
          />
        </div>

        <div class="col-md-4">
          <Members
            name="Deaka Rizkia Rahma"
            major="Information System"
            role="Official"
            instagram="dearrizk_"
            image="/members/4.png"
          />
        </div>
        <div class="col-md-4">
          <Members
            name="Muhammad Fahridho Nurrahman"
            major="Electrical Engineering"
            role="Electrical"
            instagram="fahridhi"
            image="/members/3.png"
          />
        </div>
        <div class="col-md-4">
          <Members
            name="M. Afif Shofwan M."
            major="Telecommunication Engineering"
            role="Official"
            instagram="afif_shofwan06"
            image="/members/6.png"
          />
        </div>
        <div class="col-md-4">
          <Members
            name="Galuh Rifa'i Najmudin"
            major="Marine Engineering"
            role="Mechanical"
            instagram="gluhrn"
            image="/members/5.png"
          />
        </div>
      </div>
    </div>
  </div>
</template>
