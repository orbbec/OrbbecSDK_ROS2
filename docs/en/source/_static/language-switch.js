// Language switch for English version
document.addEventListener('DOMContentLoaded', function() {
    // Create language switch link
    var langSwitch = document.createElement('div');
    langSwitch.className = 'language-switch';
    langSwitch.innerHTML = '<a href="/OrbbecSDK_ROS2/zh/" title="切换到中文">中文</a>';

    // Add styles
    var style = document.createElement('style');
    style.textContent = `
        .language-switch {
            position: fixed;
            top: 10px;
            right: 60px;
            z-index: 1000;
        }
        .language-switch a {
            color: #fff;
            background: #2980b9;
            padding: 5px 15px;
            border-radius: 3px;
            text-decoration: none;
            font-size: 14px;
            transition: background 0.3s;
        }
        .language-switch a:hover {
            background: #3498db;
        }
        @media screen and (max-width: 768px) {
            .language-switch {
                top: 5px;
                right: 10px;
            }
        }
    `;

    document.head.appendChild(style);
    document.body.appendChild(langSwitch);
});
