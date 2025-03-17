---
layout: default
title: META-SEJONG AI Robotics Challenge
lang: en
ref: home
permalink: /en/
---
{% assign t = site.data.translations.en %}

<nav class="section-nav">
  <a href="#intro-section">{{ t.nav.home }}</a>
  <a href="#environment-section">Environment</a>
  <a href="#objectives-section">Objectives</a>
  <a href="#api-section">APIs</a>
  <a href="#evaluation-section">Evaluation</a>
</nav>

<div class="title">{{ t.home.event.title }}</div>

<div class="title-event"><a href="https://ieee-metacom.org">IEEE MetaCom 2025</a></div>

<section id="intro-section">
  <div class="title-level-1">{{ t.home.intro-section.title }}</div>

  {% assign intro = t.home.intro-section.content %}
  <p>{{ intro.text-1 }}</p>
  <p>{{ intro.text-2 }}</p>
  <p>{{ intro.text-3 }}</p>

  <p>{{ intro.text-4 }}</p>
  <ul>
    <li>{{ intro.text-4-1 }}</li>
    <li>{{ intro.text-4-2 }}</li>
    <li>{{ intro.text-4-3 }}</li>
    <li>{{ intro.text-4-4 }}</li>
  </ul>

  <p>{{ intro.text-5 }}</p>

  <div class="intro-video">
    <img src="/assets/images/meta-sejong.png" alt="Competition Introduction Video" style="width: 100%; height: 100%; object-fit: cover;">
  </div>

  <div class="links">
    <div class="link-button">
      <a href=""><i class="fab fa-github"></i> Code</a>
    </div>
    <div class="link-button">
      <a href=""><i class="fas fa-book"></i> Docs</a>
    </div>
    <div class="link-button">
      <a href=""><i class="fas fa-database"></i> Data</a>
    </div>
    <div class="link-button">
      <a href=""><i class="fas fa-video"></i> Video</a>
    </div>
  </div>
</section>

<section id="environment-section">
  <div class="title-level-1">{{ t.home.environment-section.title }}</div>
  {% assign env = t.home.environment-section.content %}
  <p>{{ env.text-1 }}</p>
  <p>{{ env.text-2 }}</p>
  <p>{{ env.text-3 }}</p>
</section>

<section id="objectives-section">
  <div class="title-level-1">{{ t.home.objectives-section.title }}</div>
  {% assign obj = t.home.objectives-section.content %}
  <p>{{ obj.text-1 }}</p>
  <p>{{ obj.text-2 }}</p>
  <p>{{ obj.text-3 }}</p>
  <p>{{ obj.text-4 }}</p>
  <p>{{ obj.text-5 }}</p>
</section>

<section id="api-section">
  <div class="title-level-1">{{ t.home.api-section.title }}</div>
  {% assign api = t.home.api-section.content %}
  <p>{{ api.text-1 }}</p>
  
  <h3>{{ api.text-2 }}</h3>
  <ul>
    <li>{{ api.text-2-1 }}</li>
    <li>{{ api.text-2-2 }}</li>
    <li>{{ api.text-2-3 }}</li>
  </ul>

  <h3>{{ api.text-3 }}</h3>
  <ul>
    <li>{{ api.text-3-1 }}</li>
    <li>{{ api.text-3-2 }}</li>
    <li>{{ api.text-3-3 }}</li>
    <li>{{ api.text-3-4 }}</li>
  </ul>

  <h3>{{ api.text-4 }}</h3>
  <ul>
    <li>{{ api.text-4-1 }}</li>
    <li>{{ api.text-4-2 }}</li>
  </ul>
</section>

<section id="evaluation-section">
  <div class="title-level-1">{{ t.home.evaluation-section.title }}</div>
  {% assign eval = t.home.evaluation-section.content %}
  <p>{{ eval.text-1 }}</p>
  <p>{{ eval.text-2 }}</p>
  <p>{{ eval.text-3 }}</p>
</section>

<section id="sponsor-section">
  <div class="title-level-1">{{ t.sponsor-section.title }}</div>
  {% assign sponsors = t.sponsor-section.content %}
  <div class="sponsors">
    {% for sponsor in sponsors %}
    <div class="sponsor">
      <a href="{{ sponsor[1].link }}">
        <img src="/assets/images/{{ sponsor[1].image }}" alt="{{ sponsor[1].name }}">
      </a>
    </div>
    {% endfor %}
  </div>
</section>
