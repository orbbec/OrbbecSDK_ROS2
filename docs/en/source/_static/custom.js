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

