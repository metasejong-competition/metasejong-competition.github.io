---
layout: default
title: META-SEJONG AI Robotics Challenge
lang: ko
ref: home
permalink: /ko/
---
{% assign t = site.data.translations.ko %}

<div class="title" id="home">{{ t.home.event.title }}</div>

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
    <video id="intro-video" controls>
      <source src="{{ '/assets/video/META-Sejong AI Robotics Competition 2025-intro-540p.mp4' | relative_url }}" type="video/mp4">
      <source src="{{ '/assets/video/META-Sejong AI Robotics Competition 2025-intro-540p.webm' | relative_url }}" type="video/webm">
      <img src="/assets/images/meta-sejong.png" alt="Competition Introduction Video" style="width: 100%; height: 100%; object-fit: cover;">
    </video>
  </div>

  <div class="links">
    <div class="link-button">
      <a href="https://github.com/metasejong-competition/metacom2025"><i class="fab fa-github"></i> Code</a>
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


<section id="mission-section">
  <div class="title-level-1">{{ t.home.mission-section.title }}</div>
  {% assign mission = t.home.mission-section.content %}
  <p>{{ mission.text-1 }}</p>

  <h3>{{ mission.text-2 }}</h3>
  <ul class="mission-selector">
    <li>{{ mission.stage-label.stage-1 }}</li>
    <li>{{ mission.stage-label.stage-2 }}</li>
    <li>{{ mission.stage-label.stage-3 }}</li>
  </ul>

  <div class="stage-details">
    {% for stage in mission.stage-content %}
      {% assign stage-key = 'stage-' | append: forloop.index %}
      {% assign stage-item = mission.stage-content[stage-key] %}

      <div class="stage-item">
        <h4>{{ stage-item.label }}</h4>
        <p>{{ stage-item.description }}</p>
        <div class="media-block">

          {% for media-item in stage-item.media %}
            {% assign media-key = 'media-' | append: forloop.index %}
            {% assign media = stage-item.media[media-key] %}

            <div class="media-content">
            {% if media.type == 'image' %}
              <img src="{{ media.url }}" alt="{{ media.subtitle }}" />
            {% elsif media.type == 'video' %}
              <video controls>
                <source src="{{ media.url }}"  alt="{{ media.subtitle }}" type="video/webm">
                  <img src="/assets/images/mission-object-detection.png" style="width: 100%; height: 100%; object-fit: cover;">
              </video>

            {% endif %}
              <div class="media-info">
                <p class="subtitle">{{ media.subtitle }}</p>
                <p class="comment"><em>{{ media._comment }}</em></p>
              </div>
            </div>
          {% endfor %}
        </div>
      </div>
    {% endfor %}      
  </div>
</section>


<section id="award-section">
  <div class="title-level-1">{{ t.home.award-section.title }}</div>
  <p>{{ t.home.award-section.content }}</p>
</section>

<section id="timeline-section">
  <div class="title-level-1">{{ t.home.timeline-section.title }}</div>
    {% assign timelines = t.home.timeline-section.content %}
  <ul>
    {% for timeline in timelines %}
      {% assign timeline-key = 'timeline-' | append: forloop.index %}

      <li class="timeline-item"><span class="date">{{ timelines[timeline-key].date }}</span> - {{ timelines[timeline-key].description }}</li>
  {% endfor %}
  </ul>
</section>



<section id="sponsor-section">
  <div class="title-level-1">{{ t.home.sponsor-section.title }}</div>
  {% assign sponsors = t.home.sponsor-section.content %}
  <div class="sponsors">
    {% for sponsor in sponsors %}
      {% assign sponsor-key = 'sponsor-' | append: forloop.index %}

    <div class="sponsor">
      <a href="{{ sponsors[sponsor-key].link }}">
        <img src="/assets/images/{{ sponsors[sponsor-key].image }}" alt="{{ sponsors[sponsor-key].name }}">
        {{ sponsors[sponsor-key].name }}
      </a>
    </div>
    {% endfor %}
  </div>
</section>