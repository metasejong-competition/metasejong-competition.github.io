source "https://rubygems.org"

# Ruby 3.4.0 이상에서 필요한 기본 gem들
gem "csv", "~> 3.2"
gem "logger", "~> 1.5"
gem "base64", "~> 0.2.0"
gem "bigdecimal", "~> 3.1.7"

gem "jekyll", "~> 4.3.2"
gem "minima", "~> 2.5"
gem "sass-embedded", "~> 1.71.1"

group :jekyll_plugins do
  gem "jekyll-feed", "~> 0.12"
  gem "jekyll-seo-tag", "~> 2.8"
  gem "jekyll-sitemap", "~> 1.4"
  gem "jekyll-include-cache", "~> 0.2.1"
end

# Windows and JRuby does not include zoneinfo files, so bundle the tzinfo-data gem
# and associated library.
platforms :mingw, :x64_mingw, :mswin, :jruby do
  gem "tzinfo", ">= 1", "< 3"
  gem "tzinfo-data"
end

# Performance-booster for watching directories on Windows
gem "wdm", "~> 0.1.1", :platforms => [:mingw, :x64_mingw, :mswin]

# Lock `http_parser.rb` gem to `v0.6.x` on JRuby builds since newer versions of the gem
# do not have a Java counterpart.
gem "http_parser.rb", "~> 0.6.0", :platforms => [:jruby] 