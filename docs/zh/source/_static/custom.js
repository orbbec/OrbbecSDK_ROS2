// custom.js  可以禁止自动折叠，但不能跳转
// document.addEventListener('DOMContentLoaded', function () {
//     // 展开所有侧边栏目录项
//     var toctreeItems = document.querySelectorAll('.toctree-l1');
//     toctreeItems.forEach(function (item) {
//         item.classList.add('current');
//     });

//     // 监听点击事件，确保点击标题时不会折叠其他标题，并且能够跳转到相应的文档部分
//     document.querySelectorAll('.toctree-l1 a').forEach(function (link) {
//         link.addEventListener('click', function (event) {
//             // 阻止默认的折叠行为
//             event.preventDefault();
//             // 展开点击的标题
//             var parentLi = this.closest('li');
//             parentLi.classList.add('current');
//             // 移除其他同级目录项的 'current' 类
//             parentLi.parentElement.children.forEach(function (sibling) {
//                 if (sibling !== parentLi) {
//                     sibling.classList.remove('current');
//                 }
//             });
//             // 跳转到相应的文档部分
//             var target = this.getAttribute('href');
//             if (target.startsWith('#')) {
//                 document.querySelector(target).scrollIntoView();
//             } else {
//                 // 如果链接不是锚点，则在新窗口或当前窗口打开链接
//                 window.open(target, '_blank');
//             }
//         });
//     });
// });

// Language switcher functionality
document.addEventListener('DOMContentLoaded', function() {
    // Create language switcher button
    var languageSwitcher = document.createElement('div');
    languageSwitcher.className = 'language-switcher';
    languageSwitcher.innerHTML = `
        <button class="language-toggle" id="languageToggle">
            <span class="language-icon">🌐</span>
            <span class="language-text">中文/EN</span>
        </button>
        <div class="language-dropdown" id="languageDropdown">
            <a href="javascript:void(0)" class="language-option" data-lang="en">English</a>
            <a href="javascript:void(0)" class="language-option" data-lang="zh">中文</a>
        </div>
    `;

    // Insert language switcher into the page
    var navbar = document.querySelector('.wy-nav-top');
    if (navbar) {
        navbar.appendChild(languageSwitcher);
    } else {
        // Fallback: insert at the beginning of body
        document.body.insertBefore(languageSwitcher, document.body.firstChild);
    }

    // Toggle dropdown
    var toggleButton = document.getElementById('languageToggle');
    var dropdown = document.getElementById('languageDropdown');

    toggleButton.addEventListener('click', function(e) {
        e.stopPropagation();
        dropdown.classList.toggle('show');
    });

    // Close dropdown when clicking outside
    document.addEventListener('click', function() {
        dropdown.classList.remove('show');
    });

    // Language switching logic
    var languageOptions = document.querySelectorAll('.language-option');
    languageOptions.forEach(function(option) {
        option.addEventListener('click', function(e) {
            e.preventDefault();
            var targetLang = this.getAttribute('data-lang');
            switchLanguage(targetLang);
        });
    });

    // Detect current language from URL
    detectCurrentLanguage();
});

function detectCurrentLanguage() {
    var path = window.location.pathname;
    var currentLang = 'en'; // default

    if (path.includes('/zh/')) {
        currentLang = 'zh';
    } else if (path.includes('/en/')) {
        currentLang = 'en';
    }

    // Highlight current language
    document.querySelectorAll('.language-option').forEach(function(option) {
        if (option.getAttribute('data-lang') === currentLang) {
            option.classList.add('active');
        }
    });
}

function switchLanguage(targetLang) {
    var currentPath = window.location.pathname;
    var newPath;

    // Get the base path and file path
    var pathParts = currentPath.split('/');
    var basePath = '';
    var filePath = '';

    // Find language code in path
    var langIndex = -1;
    for (var i = 0; i < pathParts.length; i++) {
        if (pathParts[i] === 'en' || pathParts[i] === 'zh') {
            langIndex = i;
            break;
        }
    }

    if (langIndex !== -1) {
        // Replace language code
        pathParts[langIndex] = targetLang;
        newPath = pathParts.join('/');
    } else {
        // Add language code
        // Assuming structure: /base/file.html -> /base/lang/file.html
        var lastSlashIndex = currentPath.lastIndexOf('/');
        if (lastSlashIndex !== -1) {
            basePath = currentPath.substring(0, lastSlashIndex);
            filePath = currentPath.substring(lastSlashIndex);
            newPath = basePath + '/' + targetLang + filePath;
        } else {
            newPath = '/' + targetLang + currentPath;
        }
    }

    // Navigate to new URL
    window.location.href = newPath;
}

