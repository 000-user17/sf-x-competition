import React from 'react';

const SearchIcon = ({onClick}: {onClick: (...args: any) => void}) => (
    <svg
        onClick={onClick}
        width="20px"
        height="20px"
        viewBox="0 0 20 20"
        version="1.1"
        xmlns="http://www.w3.org/2000/svg"
        xmlnsXlink="http://www.w3.org/1999/xlink">
        <g id="页面-1" stroke="none" strokeWidth="1" fill="none" fillRule="evenodd">
            <g id="算法大赛-答题记录" transform="translate(-848.000000, -220.000000)">
                <g id="编组-4" transform="translate(650.000000, 217.000000)">
                    <g id="icon-搜索" transform="translate(198.000000, 3.000000)">
                        <rect id="矩形" x="0" y="0" width="20" height="20" />
                        <path
                            d="M9.40055916,2 C13.4877593,2 16.8011183,5.31335899 16.8011183,9.40055916 C16.8011183,11.1518205 16.1928172,12.761012 15.1759832,14.0283654 L18,16.8518531 L16.8518144,18.0000388 L14.0283654,15.1759832 C12.761012,16.1928172 11.1518205,16.8011183 9.40055916,16.8011183 C5.31335899,16.8011183 2,13.4877593 2,9.40055916 C2,5.31335899 5.31335899,2 9.40055916,2 Z M9.40055916,3.6237797 C6.21014775,3.6237797 3.6237797,6.21014775 3.6237797,9.40055916 C3.6237797,12.5909706 6.21014775,15.1773386 9.40055916,15.1773386 C12.5909706,15.1773386 15.1773386,12.5909706 15.1773386,9.40055916 C15.1773386,6.21014775 12.5909706,3.6237797 9.40055916,3.6237797 Z"
                            id="形状结合"
                            fill="#797C8E"
                            fillRule="nonzero"
                        />
                    </g>
                </g>
            </g>
        </g>
    </svg>
);

export default SearchIcon;
