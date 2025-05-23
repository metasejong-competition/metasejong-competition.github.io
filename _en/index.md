---
layout: default
title: MARC(Meta-Sejong AI Robotics Challenge) 2025
lang: en
ref: home
permalink: /en/
---
{% assign t = site.data.translations.en %}

<div id="title-section">
  <div class="title" id="home">{{ t.home.event.title }}</div>
  <div class="title-event"><a href="https://ieee-metacom.org">IEEE MetaCom 2025</a></div>
</div>

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
  </ul>

  <p>{{ intro.text-5 }}</p>

  <div class="intro-video">
    <iframe width="100%" height="100%" src="https://www.youtube.com/embed/8_MlP3jJ940?si=Tu1yvUpNkCNhDtrK" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
  </div>

  <div class="links">
    <div class="link-button">
      <a href="https://github.com/metasejong-competition/metacom2025-metasejong"><i class="fab fa-github"></i> MetaSejong Platform</a>
    </div>
    <div class="link-button">
      <a href="https://github.com/metasejong-competition/metasejong-airobotics"><i class="fab fa-github"></i> Demo Application</a>
    </div>
    <div class="link-button">
      <a href="https://metasejong-competition.readthedocs.io"><i class="fas fa-book"></i> Developer Guide</a>
    </div>
  </div>
</section>

<section id="mission-section">
  <div class="title-level-1">{{ t.home.mission-section.title }}</div>
  {% assign mission = t.home.mission-section.content %}
  <p>{{ mission.text-1 }}</p>

  <h3>{{ mission.text-2 }}</h3>
  <ul class="mission-selector">
    <li><button class="stage-button" onclick="handleMissionClick('stage-1')">{{ mission.stage-label.stage-1 }}</button></li>
    <li><button class="stage-button"  onclick="handleMissionClick('stage-2')">{{ mission.stage-label.stage-2 }}</button></li>
    <li><button class="stage-button"  onclick="handleMissionClick('stage-3')">{{ mission.stage-label.stage-3 }}</button></li>
  </ul>

  <div class="stage-details">
    {% for stage in mission.stage-content %}
      {% assign stage-key = 'stage-' | append: forloop.index %}
      {% assign stage-item = mission.stage-content[stage-key] %}

      <div class="stage-item {{ stage-key }}">
        <h4>{{ stage-item.label }}</h4>
        <p>{{ stage-item.description }}</p>
        <div class="media-block">

          {% for media-item in stage-item.media %}
            {% assign media-key = 'media-' | append: forloop.index %}
            {% assign media = stage-item.media[media-key] %}

            <div class="media-content {{ media.wide }}">
              <div class="media-info">
                <p class="subtitle">{{ media.subtitle }}</p>
                <!-- <p class="comment"><em>{{ media._comment }}</em></p> -->
              </div>

            {% if media.type == 'image' %}
              <img src="{{ media.url }}" alt="{{ media.subtitle }}" />
            {% elsif media.type == 'video' %}
              <video controls>
                <source src="{{ media.url }}"  alt="{{ media.subtitle }}" type="video/webm">
                  <img src="/assets/images/mission-object-detection.png" style="width: 100%; height: 100%; object-fit: cover;">
              </video>

            {% endif %}

            </div>
          {% endfor %}
        </div>
      </div>
    {% endfor %}      
  </div>
</section>

<section id="paper-section">
  <div class="title-level-1">{{ t.home.paper-section.title }}</div>
  {% assign paper-contents = t.home.paper-section.content %}
  <p>{{ paper-contents.text-1 }}</p>
  <p>{{ paper-contents.text-2 }}</p>
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

<!--
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
-->
<section id="hosting-section">
  <div class="title-level-1">{{ t.home.hosting-section.title }}</div>
  <div class="hosting">
    <div class="organization">
      {% assign hostevent = t.home.hosting-section.content %}

      <a href="{{ hostevent.link }}">
        <h4><img src="/assets/images/link-icon.png" width="20" >{{ hostevent.name }}</h4>
      </a>
      <p>{{ hostevent.description }}</p>
    </div>
  </div>
</section>

<section id="organization-section">
  <div class="title-level-1">{{ t.home.organization-section.title }}</div>
  {% assign organizations = t.home.organization-section.content %}
  <div class="organizations">
    {% for organization in organizations %}
      {% assign organization-key = 'organization-' | append: forloop.index %}

    <div class="organization">
      <a href="{{ organizations[organization-key].link }}">
        <img src="/assets/images/{{ organizations[organization-key].image }}" alt="{{ organizations[organization-key].name }}">
      </a>
    </div>
    {% endfor %}
  </div>
</section>